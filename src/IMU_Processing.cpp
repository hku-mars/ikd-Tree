#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include "Exp_mat.h"
#include <Eigen/Eigen>
#include <common_lib.h>
#include <eigen_conversions/eigen_msg.h>
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
#include <livox_loam_kp/KeyPointPose.h>
#include <geometry_msgs/Vector3.h>

//#include "imu_processor/data_process.h"

/// *************Preconfiguration
using Sophus::SE3d;
using Sophus::SO3d;

#define MAX_INI_COUNT (50)

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudtmp(
    new pcl::PointCloud<pcl::PointXYZINormal>());

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType> > PointsList;

struct MeasureGroup
{
  sensor_msgs::PointCloud2ConstPtr lidar;
  std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Process(const MeasureGroup &meas, const KPPoseConstPtr &state_last);
  void Reset();

  // Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt);

  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);

  void UndistortPcl(const MeasureGroup &meas, const KPPoseConstPtr &state_last, PointCloudXYZI &pcl_in_out);

  ros::NodeHandle nh;

  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  const Sophus::SO3d GetRot() const;

  double scale_gravity;
  Eigen::Vector3d Lidar_offset_to_IMU;

  Eigen::Vector3d bias_acc;
  Eigen::Vector3d bias_gyr;
  Eigen::Vector3d Gravity_acc;

  Eigen::Vector3d pos_last;
  Eigen::Vector3d vel_last;
  Eigen::Matrix3d R_last;

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

  /*** Transform form lidar to imu ***/
  Sophus::SE3d T_i_l;
  //// For timestamp usage
  sensor_msgs::PointCloud2ConstPtr last_lidar_;
  sensor_msgs::ImuConstPtr last_imu_;

  /*** For gyroscope integration ***/
  double start_timestamp_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::deque<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Sophus::SO3d> v_rot_;
  std::vector<Eigen::Matrix3d> v_rot_pcl_;

  livox_loam_kp::KeyPointPose v_rot_kp_;
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
  pos_last       = Zero3d;
  vel_last       = Zero3d;
  R_last         = Eye3d;
  Gravity_acc    = Eigen::Vector3d(0, 0, G_m_s2);

  Lidar_offset_to_IMU = Eigen::Vector3d(0.05512, 0.02226, 0.0297);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");

  scale_gravity  = 1.0;

  bias_acc  = Eigen::Vector3d(0, 0, 0);
  bias_gyr  = bias_acc;
  pos_last       = Zero3d;
  vel_last       = Zero3d;
  R_last         = Eye3d;

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
  v_rot_pcl_.clear();
  v_rot_kp_.pose6D.clear();

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

void ImuProcess::IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu)
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
      /* ROS_ASSERT_MSG(dt1 <= 0 && dt2 <= 0, "%f - %f - %f",
                    last_imu_->header.stamp.toSec(), start_timestamp_,
                    imu->header.stamp.toSec()); */
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
    
    Eigen::Vector3d angvel_avr = 0.5 * (gyr + gyr_last);

    double dt_seg    = time - time_last;
    auto delta_angle = dt_seg * angvel_avr;
    auto delta_r = SO3d::exp(delta_angle);

    SO3d rot = rot_last * delta_r;

    v_imu_.push_back(imu);
    v_rot_.push_back(rot);
  }
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, const KPPoseConstPtr &state_last, PointCloudXYZI &pcl_in_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar->header.stamp.toSec();
  
  /*** sort point clouds by offset time w.r.t the head of frame ***/
  pcl::fromROSMsg(*(meas.lidar), pcl_in_out);
  std::sort(pcl_in_out.points.begin(), pcl_in_out.points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_in_out.points.back().curvature / double(1000);

  /*** initialization ***/
  v_rot_kp_.header = meas.lidar->header;
  v_rot_kp_.pose6D.clear();
  double offs_t = v_imu.front()->header.stamp.toSec() - pcl_beg_time;
  Pose6D rot_kp = set_pose6d(offs_t, Zero3d, Zero3d, Zero3d, Zero3d, vel_last, pos_last, R_last);
  v_rot_kp_.pose6D.push_back(rot_kp);
  v_rot_pcl_.clear();
  v_rot_pcl_.push_back(Eye3d);

  /*** forward pre-integration at each imu point ***/
  if (state_last != NULL)
  {
    pos_last<<VEC_FROM_ARRAY(state_last->pose6D.back().pos);
    vel_last<<VEC_FROM_ARRAY(state_last->pose6D.back().vel);
    R_last<<MAT_FROM_ARRAY(state_last->pose6D.back().rot);
  }

  Eigen::Vector3d acc_kp, angvel_avr, acc_avr, vel_kp(vel_last), vel_endpcl, pos_kp(pos_last), pos_endpcl;
  Eigen::Matrix3d R_kp(R_last), R_relat, R_endpcl(R_last);
  for (auto it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
  {
    auto &head = *(it_imu);
    auto &tail = *(it_imu + 1);
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                 0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                 0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr    <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                 0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                 0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
    angvel_avr -= bias_gyr;
    acc_avr     *= G_m_s2 / scale_gravity;
    double dt    = tail->header.stamp.toSec() - head->header.stamp.toSec();

    /* total acceleration in the place of Lidar (consider the offset of Lidar to IMU) */
    acc_kp = R_kp * (acc_avr + angvel_avr.cross(angvel_avr.cross(Lidar_offset_to_IMU))) + Gravity_acc; 

    /* position of Lidar */
    pos_kp = pos_kp + vel_kp * dt + 0.5 * acc_kp * dt * dt;

    /* velocity of Lidar */
    vel_kp = vel_kp + acc_kp * dt;

    /* attitude of Lidar */
    R_kp   = R_kp * Exp(angvel_avr, dt);
    // std::cout<<"acc_kp measures"<< acc_avr.transpose() << "in word frame: "<<acc_kp.transpose()<<std::endl;

    /* save the Lidar poses at each IMU measurements */
    double offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    Pose6D rot_kp = set_pose6d(offs_t, acc_kp, angvel_avr, Zero3d, bias_gyr, vel_kp, pos_kp, R_kp);
    v_rot_kp_.pose6D.push_back(rot_kp);
  }

  /*** calculated the pos and attitude at the last lidar point ***/
  // double  dt = pcl_end_time - imu_end_time;
  // pos_endpcl = pos_kp + vel_kp * dt + 0.5 * acc_kp * dt * dt;
  // R_endpcl   = R_kp * Exp(angvel_avr, dt);
  pos_endpcl = pos_kp;
  R_endpcl   = R_kp;
  set_array(v_rot_kp_.rot_end, R_endpcl);
  set_array(v_rot_kp_.pos_end, pos_endpcl);
  set_array(v_rot_kp_.gravity, Gravity_acc);

  /*** undistort each lidar point (backward pre-integration) ***/
  auto it_pcl = pcl_in_out.points.end() - 1;
  auto &kps = v_rot_kp_.pose6D;
  /* using the results of forward pre-integration */
  for (auto it_kp = kps.end() - 1; it_kp != kps.begin(); it_kp--)
  {
    R_kp<<MAT_FROM_ARRAY(it_kp->rot);
    acc_kp<<VEC_FROM_ARRAY(it_kp->acc);
    vel_kp<<VEC_FROM_ARRAY(it_kp->vel);
    pos_kp<<VEC_FROM_ARRAY(it_kp->pos);
    angvel_avr<<VEC_FROM_ARRAY(it_kp->gyr);

    double cur_offst = it_pcl->curvature / double(1000);
    double dt = it_kp->offset_time - cur_offst;
    int i = 0;
    for(; (cur_offst > (it_kp - 1)->offset_time); it_pcl --)
    {
      cur_offst = it_pcl->curvature / double(1000);
      dt        = it_kp->offset_time - cur_offst;

      // i++; if(i % 20 == 0)  {std::cout<<"~~~~~~~dt:  "<<dt<<std::endl;}
      
      /* Transform to the 'end' frame, using only the rotation
         Note: Compensation direction is INVERSE of Frame's moving direction
         So if we want to compensate a point at timestamp-i to the frame-e
         P_compensate = Exp(omega, dt) * R_last * Pi + t_ei */

      Eigen::Vector3d point_cur(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d tie(0, 0, 0);// Eigen::Vector3d tie(- acc_avr * dt + T_kp);
      Eigen::Matrix3d Rie(R_endpcl.transpose() * R_kp * Exp(angvel_avr, - dt));
      Eigen::Vector3d v_pt_comp_e = Rie * point_cur + tie;

      /// save Undistorted points and their rotation
      it_pcl->x = v_pt_comp_e.x();
      it_pcl->y = v_pt_comp_e.y();
      it_pcl->z = v_pt_comp_e.z();

      v_rot_pcl_.push_back(Rie);
      if (it_pcl == pcl_in_out.points.begin()) break;
    }
  }
  // ROS_INFO("undistort rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
  //          v_rot_pcl_.back().eulerAngles(0, 1, 2)[0] * 180.0 / M_PI,
  //          v_rot_pcl_.back().eulerAngles(0, 1, 2)[1] * 180.0 / M_PI,
  //          v_rot_pcl_.back().eulerAngles(0, 1, 2)[2] * 180.0 / M_PI);
  // std::cout<< "v_rot_pcl_ size: "<<v_rot_pcl_.size()<<"v_rot_kp_ size: "<< v_rot_kp_.pose6D.size()<<"v_rot_ size: "<< v_rot_.size()<< std::endl;
}

void ImuProcess::Process(const MeasureGroup &meas, const KPPoseConstPtr &state_last)
{
  clock_t process_start, t1,t2,t3;

  process_start=clock();

  ROS_ASSERT(!meas.imu.empty());
  ROS_ASSERT(meas.lidar != nullptr);
  ROS_INFO("Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
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

    Eigen::Vector3d cur_acc(last_imu_->linear_acceleration.x, last_imu_->linear_acceleration.y,
                        last_imu_->linear_acceleration.z);
    Eigen::Vector3d cur_gyr(last_imu_->angular_velocity.x, last_imu_->angular_velocity.y,
                      last_imu_->angular_velocity.z);
    double cur_norm = cur_acc.norm();

    init_iter_num = 1;
    scale_gravity = cur_norm;
    mean_acc      = cur_acc;
    mean_gyr      = cur_gyr;

    ROS_WARN("The very first lidar frame");

    /// Do nothing more, return
    b_first_frame_ = false;

    return;
  }

  if (Need_init)
  {
    /// 1. initializing the gyro bias, acc and gyro covariance
    /// 2. normalize the acceleration measurenments to unit gravity

    double cur_norm = 1.0;
    int N           = init_iter_num;
    ROS_INFO("IMU Initializing: %.1f %%", float(N) / MAX_INI_COUNT * 100);

    for (const auto &imu : meas.imu)
    {
      Eigen::Vector3d cur_acc(imu->linear_acceleration.x, imu->linear_acceleration.y,
                        imu->linear_acceleration.z);
      Eigen::Vector3d cur_gyr(imu->angular_velocity.x, imu->angular_velocity.y,
                        imu->angular_velocity.z);
      cur_norm = cur_acc.norm();

      N = init_iter_num;

      scale_gravity += (cur_norm - scale_gravity) / N;
      mean_acc      += (cur_acc - mean_acc) / N;
      mean_gyr      += (cur_gyr - mean_gyr) / N;

      cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
      cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

      init_iter_num ++;
      // std::cout<< imu->linear_acceleration_covariance[0] <<std::endl;
    }

    if (init_iter_num > MAX_INI_COUNT)
    {
      Need_init   = false;
      Gravity_acc = - mean_acc /scale_gravity * G_m_s2;
      R_last      = Eye3d;// Exp(mean_acc.cross(Eigen::Vector3d(0,0,-1/scale_gravity)));
      bias_gyr    = mean_gyr;

      std::cout<<"mean acc: "<<mean_acc<<" acc measures in word frame:"<<R_last.transpose()*mean_acc<<std::endl;
      ROS_INFO("Calibration Results: Gravity_scale: %.4f; bias_gyr: %.4f %.4f %.4f; acc covarience: %.4f %.4f %.4f; gry covarience: %.4f %.4f %.4f",\
               scale_gravity, bias_gyr[0], bias_gyr[1], bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
    }
  }
  else
  {
    ROS_INFO("Process IMU");
    /// Integrate all input imu message
    // IntegrateGyr(meas.imu);

    /// Initial pose from IMU (with only rotation)
    t1 = clock();

    /// Undistort points： the first point is assummed as the base frame
    /// Compensate lidar points with IMU rotation (with only rotation now)
    UndistortPcl(meas, state_last, *cur_pcl_un_);

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

    {
      static ros::Publisher pub_KeyPointPose6D =
          nh.advertise<livox_loam_kp::KeyPointPose>("/Pose6D_IMUKeyPoints", 100);
      pub_KeyPointPose6D.publish(v_rot_kp_);
    }

    t3 = clock();
    
    std::cout<<"IMU Processing Time: preintegration "\
    <<t1 - process_start<<" undistort "<<t2 - t1<<" publish "<<t3 - t2<<std::endl;

    /// Record last measurements
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();
    cur_pcl_in_.reset(new PointCloudXYZI());
    cur_pcl_un_.reset(new PointCloudXYZI());
  }
}

/// *************ROS Node
/// To notify new data
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
bool b_exit = false;
bool b_reset = false;

/// Buffers for measurements
double last_timestamp_lidar = -1;
double last_timestamp_imu   = -1;
double last_timestamp_odo   = -1;
double last_timestamp_pose  = -1;

std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
std::deque<nav_msgs::Odometry::ConstPtr> odo_buffer;
std::deque<livox_loam_kp::KeyPointPoseConstPtr> pose_buffer;

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

  if (timestamp < last_timestamp_lidar)
  {
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

void odo_cbk(const nav_msgs::Odometry::ConstPtr &msg_in)
{
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry(*msg_in));
  double timestamp = msg->header.stamp.toSec();
  // ROS_DEBUG("get imu at time: %.6f", timestamp);
  mtx_buffer.lock();

  if (timestamp < last_timestamp_odo) {
    ROS_ERROR("odometry loop back, clear buffer");
    odo_buffer.clear();
  }
  last_timestamp_odo = timestamp;

  odo_buffer.push_back(msg);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void pose_cbk(const livox_loam_kp::KeyPointPoseConstPtr& KeyPointPose)
{
    pose_buffer.push_back(KeyPointPose);
    last_timestamp_pose = pose_buffer.front()->header.stamp.toSec();
}

bool SyncMeasure(MeasureGroup &measgroup, KPPoseConstPtr& state_last) 
{
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    /// Note: this will happen
    // ROS_INFO("NO IMU DATA");
    return false;
  }

  if (imu_buffer.back()->header.stamp.toSec() <
      lidar_buffer.front()->header.stamp.toSec()) 
  {
    return false;
  }

  /// Add lidar data, and pop from buffer
  measgroup.lidar = lidar_buffer.front();
  lidar_buffer.pop_front();
  pcl::PointCloud<PointType> v_pcl;
  pcl::fromROSMsg(*(measgroup.lidar), v_pcl);

  // double lidar_end_time = measgroup.lidar->header.stamp.toSec();
  double lidar_end_time = measgroup.lidar->header.stamp.toSec() + \
                          v_pcl.points.back().curvature / double(1000);

  /// Add imu data, and pop from buffer
  measgroup.imu.clear();
  int imu_cnt = 0;
  for (const auto &imu : imu_buffer)
  {
    double imu_time = imu->header.stamp.toSec();
    if (imu_time <= lidar_end_time) 
    {
      measgroup.imu.push_back(imu);
      imu_cnt++;
    }
  }

  for (int i = 0; i < imu_cnt; ++i)
  {
    imu_buffer.pop_front();
  }

  if(!pose_buffer.empty())
  {
    state_last = pose_buffer.back();
    pose_buffer.pop_front();
    std::cout<<"state_last time: "<<state_last->header.stamp.toSec()<<" pos: "<<state_last->pose6D.back().pos[2]<<std::endl;
  }
  
  return true;
}

void ProcessLoop(std::shared_ptr<ImuProcess> p_imu)
{
  ROS_INFO("Start ProcessLoop");

  ros::Rate r(100);

  while (ros::ok())
  {
    MeasureGroup meas;
    KPPoseConstPtr state_last;
    std::unique_lock<std::mutex> lk(mtx_buffer);
    // ROS_INFO("wait imu");
    sig_buffer.wait(lk,
                    [&meas, &state_last]() -> bool { return SyncMeasure(meas, state_last) || b_exit; });
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
    p_imu->Process(meas, state_last);

    r.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_process");
  ros::NodeHandle nh;
  signal(SIGINT, SigHandle);

  ros::Subscriber sub_pcl = nh.subscribe("/laser_cloud_flat", 100, pointcloud_cbk);
  ros::Subscriber sub_imu = nh.subscribe("/livox/imu", 100, imu_cbk);
  ros::Subscriber sub_odo = nh.subscribe("/aft_mapped_to_init", 10, odo_cbk); //pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 10);
  ros::Subscriber sub_pose = nh.subscribe("/Pose6D_Solved", 10, pose_cbk);
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
