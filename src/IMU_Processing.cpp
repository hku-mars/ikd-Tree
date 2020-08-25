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

/// *************Preconfiguration

#define MAX_INI_COUNT (50)
#define BIAS_COV (0.01)
#define INIT_COV (0.01)

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

  void Process(const MeasureGroup &meas, const KPPoseConstPtr &state_in);
  void Reset();
  void IMU_Initial(const MeasureGroup &meas, int &N);

  // Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt);

  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);

  void UndistortPcl(const MeasureGroup &meas, const KPPoseConstPtr &state_in, PointCloudXYZI &pcl_in_out);

  ros::NodeHandle nh;

  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  double scale_gravity;
  Eigen::Vector3d Lidar_offset_to_IMU;

  Eigen::Vector3d bias_acc;
  Eigen::Vector3d bias_gyr;
  Eigen::Vector3d Gravity_acc;

  Eigen::Vector3d pos_last;
  Eigen::Vector3d vel_last;
  Eigen::Matrix3d R_last;
  Eigen::Matrix<double,DIM_OF_STATES,DIM_OF_STATES> cov_state_last;
  Eigen::Matrix<double,DIM_OF_PROC_N,1> cov_proc_noise;

  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;

 private:
  /*** Whether is the first frame, init for first frame ***/
  bool b_first_frame_ = true;
  bool imu_need_init_      = true;

  int init_iter_num = 1;
  Eigen::Vector3d mean_acc;
  Eigen::Vector3d mean_gyr;

  /*** Input pointcloud ***/
  PointCloudXYZI::Ptr cur_pcl_in_;
  /*** Undistorted pointcloud ***/
  PointCloudXYZI::Ptr cur_pcl_un_;

  //// For timestamp usage
  sensor_msgs::PointCloud2ConstPtr last_lidar_;
  sensor_msgs::ImuConstPtr last_imu_;

  /*** For gyroscope integration ***/
  double start_timestamp_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::deque<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Eigen::Matrix3d> v_rot_pcl_;

  livox_loam_kp::KeyPointPose v_rot_kp_;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), last_lidar_(nullptr), last_imu_(nullptr), start_timestamp_(-1)
{
  Eigen::Quaterniond q(0, 1, 0, 0);
  Eigen::Vector3d t(0, 0, 0);
  init_iter_num = 1;
  scale_gravity = 1.0;
  cov_acc       = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr       = Eigen::Vector3d(0.1, 0.1, 0.1);
  mean_acc      = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr      = Eigen::Vector3d(0, 0, 0);
  pos_last      = Zero3d;
  vel_last      = Zero3d;
  R_last        = Eye3d;
  Gravity_acc   = Eigen::Vector3d(0, 0, G_m_s2);
  cov_state_last = Eigen::Matrix<double,DIM_OF_STATES,DIM_OF_STATES>::Identity() * INIT_COV;
  cov_proc_noise = Eigen::Matrix<double,DIM_OF_PROC_N,1>::Zero();
  Lidar_offset_to_IMU = Eigen::Vector3d(0.05512, 0.02226, 0.0297);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");

  scale_gravity  = 1.0;

  bias_acc  = Eigen::Vector3d(0, 0, 0);
  bias_gyr  = Eigen::Vector3d(0, 0, 0);
  pos_last  = Zero3d;
  vel_last  = Zero3d;
  R_last    = Eye3d;
  cov_state_last = Eigen::Matrix<double,DIM_OF_STATES,DIM_OF_STATES>::Identity() * INIT_COV;
  cov_proc_noise = Eigen::Matrix<double,DIM_OF_PROC_N,1>::Zero();

  cov_acc   = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr   = Eigen::Vector3d(0.1, 0.1, 0.1);
  mean_acc  = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr  = Eigen::Vector3d(0, 0, 0);

  imu_need_init_ = true;
  b_first_frame_ = true;
  init_iter_num  = 1;

  last_lidar_    = nullptr;
  last_imu_      = nullptr;

  //gyr_int_.Reset(-1, nullptr);
  start_timestamp_ = -1;
  v_imu_.clear();
  v_rot_pcl_.clear();
  v_rot_kp_.pose6D.clear();

  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::IMU_Initial(const MeasureGroup &meas, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  Eigen::Vector3d cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    double &&cur_norm = cur_acc.norm();

    scale_gravity = cur_acc.norm();
    mean_acc      = cur_acc;
    mean_gyr      = cur_gyr;

    N = 1;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    double &&cur_norm = cur_acc.norm();

    scale_gravity += (cur_norm - scale_gravity) / N;
    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    N ++;
  }

  Gravity_acc = - mean_acc /scale_gravity * G_m_s2;
  R_last      = Eye3d;// Exp(mean_acc.cross(Eigen::Vector3d(0,0,-1/scale_gravity)));
  bias_gyr    = mean_gyr;
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, const KPPoseConstPtr &state_in, PointCloudXYZI &pcl_in_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar->header.stamp.toSec();
  
  /*** sort point clouds by offset time ***/
  pcl::fromROSMsg(*(meas.lidar), pcl_in_out);
  std::sort(pcl_in_out.points.begin(), pcl_in_out.points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_in_out.points.back().curvature / double(1000);
  std::cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
           <<meas.imu.size()<<" imu msgs from "<<meas.imu.front()->header.stamp.toSec()<<" to " \
           <<meas.imu.back()->header.stamp.toSec()<<std::endl;

  /*** initialization ***/
  if (state_in != NULL)
  {
    KPPose state(*state_in);
    Gravity_acc<<VEC_FROM_ARRAY(state_in->gravity);
    bias_gyr<<VEC_FROM_ARRAY(state_in->bias_gyr);
    bias_acc<<VEC_FROM_ARRAY(state_in->bias_acc);
    pos_last<<VEC_FROM_ARRAY(state.pos_end);
    vel_last<<VEC_FROM_ARRAY(state.vel_end);
    R_last  =Eigen::Map<Eigen::Matrix3d>(state.rot_end.data());
    cov_state_last = Eigen::Map<Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> >(state.cov.data());
    
    Eigen::Vector3d euler_cur = correct_pi(R_last.eulerAngles(1, 0, 2));
    // std::cout<<"!!!! Got pose: R: "<<euler_cur.transpose()<<" T: "<<pos_last.transpose()<<std::endl;
  }
  v_rot_kp_.header  = meas.lidar->header;
  v_rot_kp_.pose6D.clear();
  v_rot_kp_.pose6D.push_back(set_pose6d(0.0, Zero3d, Zero3d, vel_last, pos_last, R_last));
  v_rot_pcl_.clear();
  v_rot_pcl_.push_back(Eye3d);

  /*** forward pre-integration at each imu point ***/
  Eigen::Vector3d acc_kp, angvel_avr, acc_avr, vel_kp(vel_last), vel_e, pos_kp(pos_last), pos_e, pos_relat;
  Eigen::Matrix3d R_kp(R_last), R_relat, R_e;
  Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
  Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
  double dt = 0;
  for (auto it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
    angvel_avr -= bias_gyr;
    acc_avr     = acc_avr * G_m_s2 / scale_gravity;
    /* we propagate from the first lidar point to the last imu point */
    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    
    /* covariance propagation */
    Eigen::Matrix3d acc_avr_skew;
    auto Exp_f = Exp(angvel_avr, dt);
    acc_avr_skew<<SKEW_SYM_MATRX(angvel_avr);

    F_x.block<3,3>(0,0)  = Exp_f;
    F_x.block<3,3>(0,9)  = - Eye3d * dt;
    F_x.block<3,3>(3,6)  = Eye3d * dt;
    F_x.block<3,3>(6,0)  = R_kp * acc_avr_skew * dt;
    F_x.block<3,3>(6,12) = - R_kp * dt;
    F_x.block<3,3>(6,15) = - Eye3d * dt;

    cov_w.block<3,3>(0,0).diagonal()   = cov_gyr * dt * dt;
    cov_w.block<3,3>(6,6).diagonal()   = cov_acc * dt * dt;
    cov_w.block<3,3>(9,9).diagonal()   = Eigen::Vector3d(BIAS_COV, BIAS_COV, BIAS_COV) * dt * dt;
    cov_w.block<3,3>(12,12).diagonal() = Eigen::Vector3d(BIAS_COV, BIAS_COV, BIAS_COV) * dt * dt;

    cov_state_last = F_x * cov_state_last * F_x.transpose() + cov_w;

    /* the estimated total acceleration (global frame) at the Lidar origin point (considering the offset of Lidar to IMU) */
    acc_kp = R_kp * (acc_avr + angvel_avr.cross(angvel_avr.cross(Lidar_offset_to_IMU))) + Gravity_acc; 

    /* position of Lidar */
    pos_kp = pos_kp + vel_kp * dt + 0.5 * acc_kp * dt * dt;

    /* velocity of Lidar */
    vel_kp = vel_kp + acc_kp * dt;

    /* attitude of Lidar */
    R_kp = R_kp * Exp_f;
    // std::cout<<"acc_kp measures"<< acc_avr.transpose() << "in word frame: "<<acc_kp.transpose()<<std::endl;

    /* save the Lidar poses at each IMU measurements */
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    v_rot_kp_.pose6D.push_back(set_pose6d(offs_t, acc_kp, angvel_avr, vel_kp, pos_kp, R_kp));
  }

  /*** calculated the pos and attitude at the end lidar point ***/
  dt    = pcl_end_time - imu_end_time;
  vel_e = vel_kp + acc_kp * dt;
  pos_e = pos_kp + vel_kp * dt + 0.5 * acc_kp * dt * dt;
  R_e   = R_kp * Exp(angvel_avr, dt);
  // pos_e = pos_kp;
  // R_e   = R_kp;
  v_rot_kp_.gravity  = STD_VEC_FROM_EIGEN(Gravity_acc);
  v_rot_kp_.bias_gyr = STD_VEC_FROM_EIGEN(bias_gyr);
  v_rot_kp_.bias_acc = STD_VEC_FROM_EIGEN(bias_acc);
  v_rot_kp_.pos_end  = STD_VEC_FROM_EIGEN(pos_e);
  v_rot_kp_.vel_end  = STD_VEC_FROM_EIGEN(vel_e);
  v_rot_kp_.rot_end  = STD_VEC_FROM_EIGEN(R_e);
  v_rot_kp_.cov = STD_VEC_FROM_EIGEN(cov_state_last); // std::vector<decltype(cov_state_last)::Scalar> (cov_state_last.data(), cov_state_last.data() + DIM_OF_STATES_SQUARE);
  
  Eigen::Vector3d euler_cur = correct_pi(R_e.eulerAngles(1, 0, 2));
  std::cout<<"!!!! propagated states: bg "<<cov_gyr.transpose()<<std::endl;

  /*** undistort each lidar point (backward pre-integration) ***/
  auto it_pcl = pcl_in_out.points.end() - 1;
  auto &&kps = v_rot_kp_.pose6D;
  /* using the results of forward pre-integration */
  for (auto it_kp = kps.end() - 1; it_kp != kps.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    R_kp<<MAT_FROM_ARRAY(head->rot);
    acc_kp<<VEC_FROM_ARRAY(head->acc);
    vel_kp<<VEC_FROM_ARRAY(head->vel);
    pos_kp<<VEC_FROM_ARRAY(head->pos);
    angvel_avr<<VEC_FROM_ARRAY(head->gyr);

    int i = 0;
    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;
      // i++; if (i % 50 == 1)  {std::cout<<"~~~~~~~dt: "<<dt<<" "<<it_pcl->curvature / double(1000) + pcl_beg_time<<std::endl;}
      
      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d T_ei(pos_kp + vel_kp * dt + 0.5 * acc_kp * dt * dt - pos_e);
      Eigen::Matrix3d R_i(R_kp * Exp(angvel_avr, dt));
      Eigen::Vector3d P_compensate = R_e.transpose() * (R_i * P_i + T_ei);

      /// save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      v_rot_pcl_.push_back(R_i);
      if (it_pcl == pcl_in_out.points.begin())
      {break;}
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas, const KPPoseConstPtr &state_in)
{
  double process_start, t1,t2,t3;
  t1 = omp_get_wtime();

  process_start = omp_get_wtime();

  ROS_ASSERT(!meas.imu.empty());
  ROS_ASSERT(meas.lidar != nullptr);

  auto pcl_in_msg = meas.lidar;

  if (b_first_frame_ || imu_need_init_)
  {
    /// The very first lidar frame
    IMU_Initial(meas, init_iter_num);

    imu_need_init_ = true;
    b_first_frame_ = false;
    
    last_lidar_ = pcl_in_msg;
    last_imu_   = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT)
    {
      imu_need_init_ = false;
      // std::cout<<"mean acc: "<<mean_acc<<" acc measures in word frame:"<<R_last.transpose()*mean_acc<<std::endl;
      ROS_INFO("IMU Initial Results: Gravity_scale: %.4f; bias_gyr: %.4f %.4f %.4f; acc covarience: %.4f %.4f %.4f; gry covarience: %.4f %.4f %.4f",\
               scale_gravity, bias_gyr[0], bias_gyr[1], bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
    }

    return;
  }

  /// Undistort points： the first point is assummed as the base frame
  /// Compensate lidar points with IMU rotation (with only rotation now)
  UndistortPcl(meas, state_in, *cur_pcl_un_);

  t2 = omp_get_wtime();

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

  /// Record last measurements
  last_lidar_ = pcl_in_msg;
  last_imu_ = meas.imu.back();
  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());

  t3 = omp_get_wtime();
  
  std::cout<<"[ IMU Process ]: IMU Processing Time: undistort "<<t2 - t1<<" total "<<t3 - t1<<std::endl;
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

bool SyncMeasure(MeasureGroup &measgroup, KPPoseConstPtr& state_in) 
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
    state_in = pose_buffer.back();
    pose_buffer.pop_front();
    // std::cout<<"state_in time: "<<state_in->header.stamp.toSec()<<" pos: "<<state_in->pose6D.back().pos[2]<<std::endl;
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
    KPPoseConstPtr state_in;
    std::unique_lock<std::mutex> lk(mtx_buffer);
    // ROS_INFO("wait imu");
    sig_buffer.wait(lk,
                    [&meas, &state_in]() -> bool { return SyncMeasure(meas, state_in) || b_exit; });
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
    p_imu->Process(meas, state_in);

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
