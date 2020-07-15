#include <cmath>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

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

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudtmp(
    new pcl::PointCloud<pcl::PointXYZINormal>());

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

struct MeasureGroup {
  sensor_msgs::PointCloud2ConstPtr lidar;
  std::vector<sensor_msgs::Imu::ConstPtr> imu;
};

/// *************Gyroscope integratioin
class GyrInt {
 public:
  GyrInt();
  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  const Sophus::SO3d GetRot() const;

 private:
  // Sophus::SO3d r_;
  /// last_imu_ is
  double start_timestamp_;
  sensor_msgs::ImuConstPtr last_imu_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::vector<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Sophus::SO3d> v_rot_;
};

GyrInt::GyrInt() : start_timestamp_(-1), last_imu_(nullptr) {}

void GyrInt::Reset(double start_timestamp,
                   const sensor_msgs::ImuConstPtr &lastimu) {
  start_timestamp_ = start_timestamp;
  last_imu_ = lastimu;

  v_rot_.clear();
  v_imu_.clear();
}

const Sophus::SO3d GyrInt::GetRot() const {
  if (v_rot_.empty()) {
    return SO3d();
  } else {
    return v_rot_.back();
  }
}

void GyrInt::Integrate(const sensor_msgs::ImuConstPtr &imu) {
  /// Init
  if (v_rot_.empty()) {
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

  ///
  const SO3d &rot_last = v_rot_.back();
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

/// *************IMU Process and undistortion
class ImuProcess {
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

 private:
  /*** Whether is the first frame, init for first frame ***/
  bool b_first_frame_ = true;

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
  GyrInt gyr_int_;
};


ImuProcess::ImuProcess()
    : b_first_frame_(true), last_lidar_(nullptr), last_imu_(nullptr) {
  Eigen::Quaterniond q(0, 1, 0, 0);
  Eigen::Vector3d t(0, 0, 0);
  T_i_l = Sophus::SE3d(q, t);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
  ROS_WARN("Reset ImuProcess");

  b_first_frame_ = true;
  last_lidar_ = nullptr;
  last_imu_ = nullptr;

  gyr_int_.Reset(-1, nullptr);

  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::IntegrateGyr(
    const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu) {
  /// Reset gyr integrator
  gyr_int_.Reset(last_lidar_->header.stamp.toSec(), last_imu_);
  /// And then integrate all the imu measurements
  for (const auto &imu : v_imu) {
    gyr_int_.Integrate(imu);
  }
  ROS_INFO("integrate rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
           gyr_int_.GetRot().angleX() * 180.0 / M_PI,
           gyr_int_.GetRot().angleY() * 180.0 / M_PI,
           gyr_int_.GetRot().angleZ() * 180.0 / M_PI);
}

void ImuProcess::UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out,
                              double dt_be, const Sophus::SE3d &Tbe) {
  const Eigen::Vector3d &tbe = Tbe.translation();
  Eigen::Vector3d rso3_be = Tbe.so3().log();
  for (auto &pt : pcl_in_out->points) {
    int ring = int(pt.intensity);
    float dt_bi = pt.intensity - ring;

    if (dt_bi == 0) laserCloudtmp->push_back(pt);
    double ratio_bi = dt_bi / dt_be;
    /// Rotation from i-e
    double ratio_ie = 1 - ratio_bi;

    Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
    SO3d Rie = SO3d::exp(rso3_ie);

    /// Transform to the 'end' frame, using only the rotation
    /// Note: Compensation direction is INVERSE of Frame's moving direction
    /// So if we want to compensate a point at timestamp-i to the frame-e
    /// P_compensate = R_ei * Pi + t_ei
    Eigen::Vector3d tie = ratio_ie * tbe;
    // Eigen::Vector3d tei = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
    Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

    /// Undistorted point
    pt.x = v_pt_comp_e.x();
    pt.y = v_pt_comp_e.y();
    pt.z = v_pt_comp_e.z();
  }
}

void ImuProcess::Process(const MeasureGroup &meas) {
  ROS_INFO("Process IMU");
  ROS_ASSERT(!meas.imu.empty());
  ROS_ASSERT(meas.lidar != nullptr);
  ROS_INFO("Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
            meas.lidar->header.stamp.toSec(), meas.imu.size(),
            meas.imu.front()->header.stamp.toSec(),
            meas.imu.back()->header.stamp.toSec());

  auto pcl_in_msg = meas.lidar;

  if (b_first_frame_) {
    /// The very first lidar frame

    /// Reset
    Reset();

    /// Record first lidar, and first useful imu
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();

    ROS_WARN("The very first lidar frame");

    /// Do nothing more, return
    b_first_frame_ = false;
    return;
  }

  /// Integrate all input imu message
  IntegrateGyr(meas.imu);

  /// Compensate lidar points with IMU rotation
  //// Initial pose from IMU (with only rotation)
  SE3d T_l_c(gyr_int_.GetRot(), Eigen::Vector3d::Zero());
  dt_l_c_ = pcl_in_msg->header.stamp.toSec() - last_lidar_->header.stamp.toSec();
  //// Get input pcl
  pcl::fromROSMsg(*pcl_in_msg, *cur_pcl_in_);

  /// Undistort points

  Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;
  pcl::copyPointCloud(*cur_pcl_in_, *cur_pcl_un_);
  UndistortPcl(cur_pcl_un_, dt_l_c_, T_l_be);

  {
    static ros::Publisher pub_UndistortPcl =
        nh.advertise<sensor_msgs::PointCloud2>("/livox_first_point", 100);
    sensor_msgs::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(*laserCloudtmp, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "/camera_init";
    pub_UndistortPcl.publish(pcl_out_msg);
    laserCloudtmp->clear();
  }

  {
    static ros::Publisher pub_UndistortPcl =
        nh.advertise<sensor_msgs::PointCloud2>("/livox_undistort", 100);
    sensor_msgs::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "/camera_init";
    pub_UndistortPcl.publish(pcl_out_msg);
  }

  {
    static ros::Publisher pub_UndistortPcl =
        nh.advertise<sensor_msgs::PointCloud2>("/livox_distort", 100);
    sensor_msgs::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(*cur_pcl_in_, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "/camera_init";
    pub_UndistortPcl.publish(pcl_out_msg);
  }

  /// Record last measurements
  last_lidar_ = pcl_in_msg;
  last_imu_ = meas.imu.back();
  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

/// *************ROS Node
std::string topic_pcl = "/livox/lidar";
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

void SigHandle(int sig) {
  b_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

void pointcloud_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  const double timestamp = msg->header.stamp.toSec();
  // ROS_DEBUG("get point cloud at time: %.6f", timestamp);

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

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
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

bool SyncMeasure(MeasureGroup &measgroup) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    /// Note: this will happen
    ROS_INFO("NO IMU DATA");
    return false;
  }
  ROS_INFO("IMU DATA step 1");
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
  for (const auto &imu : imu_buffer) {
    double imu_time = imu->header.stamp.toSec();
    if (imu_time <= lidar_time) {
      measgroup.imu.push_back(imu);
      imu_cnt++;
    }
  }
  for (int i = 0; i < imu_cnt; ++i) {
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

  ros::Subscriber sub_pcl = nh.subscribe(topic_pcl, 10, pointcloud_cbk);
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
