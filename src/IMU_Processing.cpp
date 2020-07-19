#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

//#include "imu_processor/data_process.h"

/// *************Preconfiguration
using Sophus::SE3d;
using Sophus::SO3d;

#define SKEW_SYM_MATRX(v)  0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudtmp(
    new pcl::PointCloud<pcl::PointXYZINormal>());

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

struct MeasureGroup 
{
  sensor_msgs::PointCloud2ConstPtr lidar;
  std::vector<sensor_msgs::Imu::ConstPtr> imu;
};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Process(const MeasureGroup &meas);
  void Reset();

  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);

  void UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out, double dt_be,
                    const Sophus::SE3d &Tbe);

  ros::NodeHandle nh;

  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  const Sophus::SO3d GetRot() const;

  double scale_gravity;

  Eigen::Vector3d zero_bias_acc;
  Eigen::Vector3d zero_bias_gyr;

  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;

 private:
  /*** Whether is the first frame, init for first frame ***/
  bool b_first_frame_ = true;
  bool Need_init      = true;

  int init_iter_num = 1;
  Eigen::Vector3d mean_acc;
  Eigen::Vector3d mean_gyr;

  /*** Input pointcloud ***/
  PointCloudXYZI::Ptr cur_pcl_in_;
  /*** Undistorted pointcloud ***/
  PointCloudXYZI::Ptr cur_pcl_un_;

  double dt_l_c_;

  /*** Transform form lidar to imu ***/
  Sophus::SE3d T_i_l;
  //// For timestamp usage
  sensor_msgs::PointCloud2ConstPtr last_lidar_;
  sensor_msgs::ImuConstPtr last_imu_;

  /*** For gyroscope integration ***/
  double start_timestamp_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::vector<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Sophus::SO3d> v_rot_;
};


ImuProcess::ImuProcess()
    : b_first_frame_(true), Need_init(true), last_lidar_(nullptr), last_imu_(nullptr), start_timestamp_(-1)
{
  Eigen::Quaterniond q(0, 1, 0, 0);
  Eigen::Vector3d t(0, 0, 0);
  T_i_l = Sophus::SE3d(q, t);
  init_iter_num = 1;
  scale_gravity = 1.0;
  cov_acc       = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr       = Eigen::Vector3d(0.1, 0.1, 0.1);
  mean_acc      = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr      = Eigen::Vector3d(0, 0, 0);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");

  scale_gravity  = 1.0;

  zero_bias_acc  = Eigen::Vector3d(0, 0, 0);
  zero_bias_gyr  = zero_bias_acc;

  cov_acc        = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr        = Eigen::Vector3d(0.1, 0.1, 0.1);
  mean_acc       = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr       = Eigen::Vector3d(0, 0, 0);

  Need_init      = true;
  b_first_frame_ = true;
  init_iter_num  = 1;

  last_lidar_    = nullptr;
  last_imu_      = nullptr;

  //gyr_int_.Reset(-1, nullptr);
  start_timestamp_ = -1;
  v_rot_.clear();
  v_imu_.clear();

  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

const Sophus::SO3d ImuProcess::GetRot() const 
{
  if (v_rot_.empty())
  {
    return SO3d();
  }
  else 
  {
    return v_rot_.back();
  }
}

void ImuProcess::IntegrateGyr(
    const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu) 
{
  /// Reset gyr integrator
  // gyr_int_.Reset(last_lidar_->header.stamp.toSec(), last_imu_);
  start_timestamp_=last_lidar_->header.stamp.toSec();

  /// And then integrate all the imu measurements
  for (const auto &imu : v_imu)
  {
    if (v_rot_.empty())
    {
      ROS_ASSERT(start_timestamp_ > 0);
      ROS_ASSERT(last_imu_ != nullptr);

      /// Identity rotation
      v_rot_.push_back(SO3d());

      /// Interpolate imu in
      sensor_msgs::ImuPtr imu_inter(new sensor_msgs::Imu());
      double dt1 = start_timestamp_ - last_imu_->header.stamp.toSec();
      double dt2 = imu->header.stamp.toSec() - start_timestamp_;
      ROS_ASSERT_MSG(dt1 >= 0 && dt2 >= 0, "%f - %f - %f",
                    last_imu_->header.stamp.toSec(), start_timestamp_,
                    imu->header.stamp.toSec());
      double w1 = dt2 / (dt1 + dt2 + 1e-9);
      double w2 = dt1 / (dt1 + dt2 + 1e-9);

      const auto &gyr1 = last_imu_->angular_velocity;
      const auto &acc1 = last_imu_->linear_acceleration;
      const auto &gyr2 = imu->angular_velocity;
      const auto &acc2 = imu->linear_acceleration;

      imu_inter->header.stamp.fromSec(start_timestamp_);
      imu_inter->angular_velocity.x = w1 * gyr1.x + w2 * gyr2.x;
      imu_inter->angular_velocity.y = w1 * gyr1.y + w2 * gyr2.y;
      imu_inter->angular_velocity.z = w1 * gyr1.z + w2 * gyr2.z;
      imu_inter->linear_acceleration.x = w1 * acc1.x + w2 * acc2.x;
      imu_inter->linear_acceleration.y = w1 * acc1.y + w2 * acc2.y;
      imu_inter->linear_acceleration.z = w1 * acc1.z + w2 * acc2.z;

      v_imu_.push_back(imu_inter);
    }

    const SO3d &rot_last    = v_rot_.back();
    const auto &imumsg_last = v_imu_.back();
    const double &time_last = imumsg_last->header.stamp.toSec();
    Eigen::Vector3d gyr_last(imumsg_last->angular_velocity.x,
                            imumsg_last->angular_velocity.y,
                            imumsg_last->angular_velocity.z);
    double time = imu->header.stamp.toSec();
    Eigen::Vector3d gyr(imu->angular_velocity.x, imu->angular_velocity.y,
                        imu->angular_velocity.z);
    assert(time >= 0);
    double dt = time - time_last;
    auto delta_angle = dt * 0.5 * (gyr + gyr_last);
    auto delta_r = SO3d::exp(delta_angle);

    SO3d rot = rot_last * delta_r;

    v_imu_.push_back(imu);
    v_rot_.push_back(rot);
  }

  ROS_INFO("integrate rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
           GetRot().angleX() * 180.0 / M_PI,
           GetRot().angleY() * 180.0 / M_PI,
           GetRot().angleZ() * 180.0 / M_PI);
}

void ImuProcess::UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out,
                              double dt_be, const Sophus::SE3d &Tbe)
{
  const Eigen::Vector3d &tbe = Tbe.translation();

  // ros::Time begin, t1;
  // begin = ros::Time::now();
  Eigen::Vector3d rso3_be, r_axis;
  rso3_be = Tbe.so3().log();
  double r_angle = rso3_be.norm();
  if (r_angle > 0.00000001)
  {
    r_axis = rso3_be / r_angle;
  }
  else
  {
    r_axis = Eigen::Vector3d(0,0,0);
  }

  Eigen::Matrix3d K, Eye3d;
  
  K << SKEW_SYM_MATRX(r_axis);
  Eye3d = Eigen::Matrix3d::Identity();

  for (auto &pt : pcl_in_out->points) 
  {
    
    int ring = int(pt.intensity);
    float dt_bi = pt.intensity - ring;

    if (dt_bi == 0) {laserCloudtmp->push_back(pt);}
    double ratio_bi = dt_bi / dt_be;
    /// Rotation from i-e
    double ratio_ie = 1 - ratio_bi;

    // Eigen::Vector3d rso3_ie = -1 * ratio_ie * rso3_be;
    // SO3d Rie = SO3d::exp(rso3_ie);

    double r_angle_piece = -1 * ratio_ie * r_angle;
    Eigen::Matrix3d Rie  = Eye3d + std::sin(r_angle_piece) * K + (1.0 - std::cos(r_angle_piece)) * K * K;

    /// Transform to the 'end' frame, using only the rotation
    /// Note: Compensation direction is INVERSE of Frame's moving direction
    /// So if we want to compensate a point at timestamp-i to the frame-e
    /// P_compensate = R_ei * Pi + t_ei
    Eigen::Vector3d tie         = ratio_ie * tbe;
    Eigen::Vector3d v_pt_comp_e = Rie * (Eigen::Vector3d(pt.x, pt.y, pt.z) - tie);

    /// Undistorted point
    pt.x = v_pt_comp_e.x();
    pt.y = v_pt_comp_e.y();
    pt.z = v_pt_comp_e.z();
  }
  // ros::Duration duration1 = t1 - begin;
}

void ImuProcess::Process(const MeasureGroup &meas)
{
  clock_t process_start, t1,t2,t3;

  process_start=clock();

  ROS_ASSERT(!meas.imu.empty());
  ROS_ASSERT(meas.lidar != nullptr);
  ROS_DEBUG("Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
            meas.lidar->header.stamp.toSec(), meas.imu.size(),
            meas.imu.front()->header.stamp.toSec(),
            meas.imu.back()->header.stamp.toSec());

  auto pcl_in_msg = meas.lidar;

  if (b_first_frame_) 
  {
    /// The very first lidar frame
    Need_init = true;

    /// Reset
    Reset();

    /// Record first lidar, and first useful imu
    last_lidar_ = pcl_in_msg;
    last_imu_   = meas.imu.back();

    ROS_WARN("The very first lidar frame");

    /// Do nothing more, return
    b_first_frame_ = false;

    return;
  }

  if (Need_init)
  {
    /// 1.initializing the accelerameter and gyroscopes's covariance and bias
    /// 2.normalize the acceleration measurenments to unit gravity
    double cur_norm = 1.0;
    int N           = init_iter_num;
    ROS_INFO("IMU Initializing: %.1f %%", float(N) / 500 * 100);

    for (const auto &imu : meas.imu)
    {
      Eigen::Vector3d cur_acc(imu->linear_acceleration.x, imu->linear_acceleration.y,
                        imu->linear_acceleration.z);
      Eigen::Vector3d cur_gyr(imu->angular_velocity.x, imu->angular_velocity.y,
                        imu->angular_velocity.z);
      cur_norm = cur_acc.norm();

      if (init_iter_num <= 1)
      {
        init_iter_num = 1;
        scale_gravity = cur_norm;
        mean_acc      = cur_acc;
        mean_gyr      = cur_gyr;
      }
      else
      {
        N = init_iter_num;

        scale_gravity += (cur_norm - scale_gravity) / N;
        mean_acc      += (cur_acc - mean_acc) / N;
        mean_gyr      += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
      }
      init_iter_num ++;
    }

    if (init_iter_num>500)
    {
      scale_gravity = 1.0 / std::max(scale_gravity,0.1);
      Need_init     = false;
      ROS_INFO("Calibration Results: Gravity_scale: %.4f; acc covarience: %.4f %.4f %.4f; gry covarience: %.4f %.4f %.4f",\
               scale_gravity, cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
    }
  }
  else
  {
    ROS_INFO("Process IMU");
    /// Integrate all input imu message
    IntegrateGyr(meas.imu);

    /// Compensate lidar points with IMU rotation
    //// Initial pose from IMU (with only rotation)
    SE3d T_l_c(GetRot(), Eigen::Vector3d::Zero());
    dt_l_c_ = pcl_in_msg->header.stamp.toSec() - last_lidar_->header.stamp.toSec();
    //// Get input pcl
    pcl::fromROSMsg(*pcl_in_msg, *cur_pcl_in_);

    t1 = clock();

    /// Undistort points
    Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;
    pcl::copyPointCloud(*cur_pcl_in_, *cur_pcl_un_);
    UndistortPcl(cur_pcl_un_, dt_l_c_, T_l_be);

    t2 = clock();

    {
      static ros::Publisher pub_UndistortPcl =
          nh.advertise<sensor_msgs::PointCloud2>("/livox_first_point", 100);
      sensor_msgs::PointCloud2 pcl_out_msg;
      pcl::toROSMsg(*laserCloudtmp, pcl_out_msg);
      pcl_out_msg.header = pcl_in_msg->header;
      pcl_out_msg.header.frame_id = "/livox";
      pub_UndistortPcl.publish(pcl_out_msg);
      laserCloudtmp->clear();
    }

    {
      static ros::Publisher pub_UndistortPcl =
          nh.advertise<sensor_msgs::PointCloud2>("/livox_undistort", 100);
      sensor_msgs::PointCloud2 pcl_out_msg;
      pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
      pcl_out_msg.header = pcl_in_msg->header;
      pcl_out_msg.header.frame_id = "/livox";
      pub_UndistortPcl.publish(pcl_out_msg);
    }

    {
      static ros::Publisher pub_UndistortPcl =
          nh.advertise<sensor_msgs::PointCloud2>("/livox_distort", 100);
      sensor_msgs::PointCloud2 pcl_out_msg;
      pcl::toROSMsg(*cur_pcl_in_, pcl_out_msg);
      pcl_out_msg.header = pcl_in_msg->header;
      pcl_out_msg.header.frame_id = "/livox";
      pub_UndistortPcl.publish(pcl_out_msg);
    }

    t3 = clock();
    
    std::cout<<"Points number in one steps: "<<cur_pcl_un_->points.size()<<"; Time Consumption: preintegration "\
    <<t1 - process_start<<" undistort "<<t2 - t1<<" publish "<<t3 - t2<<std::endl;

    /// Record last measurements
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();
    cur_pcl_in_.reset(new PointCloudXYZI());
    cur_pcl_un_.reset(new PointCloudXYZI());
  }
}

/// *************ROS Node
std::string topic_pcl = "/laser_cloud_flat";
std::string topic_imu = "/livox/imu";

/// To notify new data
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
bool b_exit = false;
bool b_reset = false;

/// Buffers for measurements
double last_timestamp_lidar = -1;
std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
double last_timestamp_imu = -1;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

void SigHandle(int sig)
{
  b_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

void pointcloud_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
  const double timestamp = msg->header.stamp.toSec();
  ROS_DEBUG("get point cloud at time: %.6f", timestamp);

  mtx_buffer.lock();

  if (timestamp < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }
  last_timestamp_lidar = timestamp;

  lidar_buffer.push_back(msg);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  double timestamp = msg->header.stamp.toSec();
  // ROS_DEBUG("get imu at time: %.6f", timestamp);

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    ROS_ERROR("imu loop back, clear buffer");
    imu_buffer.clear();
    b_reset = true;
  }
  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool SyncMeasure(MeasureGroup &measgroup) 
{
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    /// Note: this will happen
    // ROS_INFO("NO IMU DATA");
    return false;
  }

  if (imu_buffer.front()->header.stamp.toSec() >
      lidar_buffer.back()->header.stamp.toSec()) {
    lidar_buffer.clear();
    ROS_ERROR("clear lidar buffer, only happen at the beginning");
    return false;
  }

  if (imu_buffer.back()->header.stamp.toSec() <
      lidar_buffer.front()->header.stamp.toSec()) {
    return false;
  }

  /// Add lidar data, and pop from buffer
  measgroup.lidar = lidar_buffer.front();
  lidar_buffer.pop_front();
  double lidar_time = measgroup.lidar->header.stamp.toSec();

  /// Add imu data, and pop from buffer
  measgroup.imu.clear();
  int imu_cnt = 0;
  for (const auto &imu : imu_buffer)
  {
    double imu_time = imu->header.stamp.toSec();
    if (imu_time <= lidar_time) {
      measgroup.imu.push_back(imu);
      imu_cnt++;
    }
  }

  for (int i = 0; i < imu_cnt; ++i)
  {
    imu_buffer.pop_front();
  }

  // ROS_DEBUG("add %d imu msg", imu_cnt);

  return true;
}

void ProcessLoop(std::shared_ptr<ImuProcess> p_imu) {
  ROS_INFO("Start ProcessLoop");

  ros::Rate r(100);
  while (ros::ok()) {
    MeasureGroup meas;
    std::unique_lock<std::mutex> lk(mtx_buffer);
    ROS_INFO("wait imu");
    sig_buffer.wait(lk,
                    [&meas]() -> bool { return SyncMeasure(meas) || b_exit; });
    lk.unlock();

    if (b_exit) {
      ROS_INFO("b_exit=true, exit");
      break;
    }

    if (b_reset) {
      ROS_WARN("reset when rosbag play back");
      p_imu->Reset();
      b_reset = false;
      continue;
    }
    p_imu->Process(meas);

    r.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "data_process");
  ros::NodeHandle nh;
  signal(SIGINT, SigHandle);

  ros::Subscriber sub_pcl = nh.subscribe(topic_pcl, 100, pointcloud_cbk);
  ros::Subscriber sub_imu = nh.subscribe(topic_imu, 100, imu_cbk);

  std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

  /// for debug
  p_imu->nh = nh;

  std::thread th_proc(ProcessLoop, p_imu);

  // ros::spin();
  ros::Rate r(1000);
  while (ros::ok()) {
    if (b_exit) break;

    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Wait for process loop exit");
  if (th_proc.joinable()) th_proc.join();

  return 0;
}
