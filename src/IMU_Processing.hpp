#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <Exp_mat.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <fast_lio/States.h>
#include <geometry_msgs/Vector3.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (50)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Process(const MeasureGroup &meas, StatesGroup &state, PointCloudXYZI::Ptr pcl_un_);
  void Reset();
  void IMU_Initial(const MeasureGroup &meas, StatesGroup &state, int &N);

  // Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt);

  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);

  void UndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_in_out);

  ros::NodeHandle nh;

  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  double scale_gravity;

  Eigen::Vector3d angvel_last;
  Eigen::Vector3d acc_s_last;

  Eigen::Matrix<double,DIM_OF_PROC_N,1> cov_proc_noise;

  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;

  std::ofstream fout;

 private:
  /*** Whether is the first frame, init for first frame ***/
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;

  int init_iter_num = 1;
  Eigen::Vector3d mean_acc;
  Eigen::Vector3d mean_gyr;

  /*** Undistorted pointcloud ***/
  PointCloudXYZI::Ptr cur_pcl_un_;

  //// For timestamp usage
  sensor_msgs::ImuConstPtr last_imu_;

  /*** For gyroscope integration ***/
  double start_timestamp_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::deque<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Eigen::Matrix3d> v_rot_pcl_;
  std::vector<Pose6D> IMUpose;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), last_imu_(nullptr), start_timestamp_(-1)
{
  Eigen::Quaterniond q(0, 1, 0, 0);
  Eigen::Vector3d t(0, 0, 0);
  init_iter_num = 1;
  scale_gravity = 1.0;
  cov_acc       = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr       = Eigen::Vector3d(0.1, 0.1, 0.1);
  mean_acc      = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr      = Eigen::Vector3d(0, 0, 0);
  angvel_last   = Zero3d;
  cov_proc_noise = Eigen::Matrix<double,DIM_OF_PROC_N,1>::Zero();
  // Lidar_offset_to_IMU = Eigen::Vector3d(0.0, 0.0, -0.0);
  // fout.open(DEBUG_FILE_DIR("imu.txt"),std::ios::out);
}

ImuProcess::~ImuProcess() {fout.close();}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");
  scale_gravity  = 1.0;
  angvel_last   = Zero3d;
  cov_proc_noise = Eigen::Matrix<double,DIM_OF_PROC_N,1>::Zero();

  cov_acc   = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr   = Eigen::Vector3d(0.1, 0.1, 0.1);
  mean_acc  = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr  = Eigen::Vector3d(0, 0, 0);

  imu_need_init_ = true;
  b_first_frame_ = true;
  init_iter_num  = 1;

  last_imu_      = nullptr;

  //gyr_int_.Reset(-1, nullptr);
  start_timestamp_ = -1;
  v_imu_.clear();
  IMUpose.clear();

  cur_pcl_un_.reset(new PointCloudXYZI());
  fout.close();

}

void ImuProcess::IMU_Initial(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  Eigen::Vector3d cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    scale_gravity += (cur_acc.norm() - scale_gravity) / N;
    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    N ++;
  }

  state_inout.gravity = - mean_acc /scale_gravity * G_m_s2;
  state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(Eigen::Vector3d(0, 0, -1 / scale_gravity)));
  state_inout.bias_g  = mean_gyr;
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time;
  
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  std::cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
           <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<std::endl;

  /*** get last states ***/
  #ifdef DEBUG_PRINT
    Eigen::Vector3d euler_cur = correct_pi(state_inout.rot_end.eulerAngles(1, 0, 2));
  #endif

  /*** Initialize IMU pose ***/
  IMUpose.clear();
  // IMUpose.push_back(set_pose6d(0.0, Zero3d, Zero3d, state.vel_end, state.pos_end, state.rot_end));
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));

  /*** forward propagation at each imu point ***/
  Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  Eigen::Matrix3d R_imu(state_inout.rot_end);
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

    angvel_avr -= state_inout.bias_g;
    acc_avr     = acc_avr * G_m_s2 / scale_gravity - state_inout.bias_a;

    #ifdef DEBUG_PRINT
    // fout<<head->header.stamp.toSec()<<" "<<angvel_avr.transpose()<<" "<<acc_avr.transpose()<<std::endl;
    #endif
    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    
    /* covariance propagation */
    Eigen::Matrix3d acc_avr_skew;
    Eigen::Matrix3d Exp_f   = Exp(angvel_avr, dt);
    acc_avr_skew<<SKEW_SYM_MATRX(angvel_avr);

    F_x.block<3,3>(0,0)  = - Exp_f;
    F_x.block<3,3>(0,9)  = - Eye3d * dt;
    // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
    F_x.block<3,3>(3,6)  = Eye3d * dt;
    F_x.block<3,3>(6,0)  = - R_imu * acc_avr_skew * dt;
    F_x.block<3,3>(6,12) = - R_imu * dt;
    F_x.block<3,3>(6,15) = Eye3d * dt;

    Eigen::Matrix3d cov_acc_diag(Eye3d), cov_gyr_diag(Eye3d);
    cov_acc_diag.diagonal() = cov_acc;
    cov_gyr_diag.diagonal() = cov_gyr;
    cov_w.block<3,3>(0,0).diagonal()   = cov_gyr * dt * dt * 100;
    cov_w.block<3,3>(3,3)              = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt * 100;
    cov_w.block<3,3>(6,6)              = R_imu * cov_acc_diag * R_imu.transpose() * dt * dt * 100;
    cov_w.block<3,3>(9,9).diagonal()   = Eigen::Vector3d(0.001, 0.001, 0.001) * dt * dt; // bias gyro covariance
    cov_w.block<3,3>(12,12).diagonal() = Eigen::Vector3d(0.001, 0.001, 0.001) * dt * dt; // bias acc covariance

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    /* propogation of Lidar attitude */
    R_imu = R_imu * Exp_f;

    /* Specific acceleration (global frame) at the IMU origin point (considering the offset of Lidar to IMU) */
    acc_imu = R_imu * acc_avr + state_inout.gravity;
    // acc_kp  = acc_imu + R_imu * angvel_avr.cross(angvel_avr.cross(Lidar_offset_to_IMU));

    /* propogation of IMU position */
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    /* velocity of IMU */
    vel_imu = vel_imu + acc_imu * dt;

    /* save the Lidar poses at each IMU measurements */
    angvel_last = angvel_avr;
    acc_s_last  = acc_imu;
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    // std::cout<<"head imu acc pre: "<<acc_imu.transpose()<<std::endl;
    IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
  }

  /*** calculated the pos and attitude prediction at the lase lidar point ***/
  dt = pcl_end_time - imu_end_time;
  state_inout.vel_end = vel_imu + acc_imu * dt;
  state_inout.rot_end = R_imu * Exp(angvel_avr, dt);
  state_inout.pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

  auto pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lidar_offset_to_IMU;

  #ifdef DEBUG_PRINT
    std::cout<<"[ IMU Process ]: vel "<<state_inout.vel_end.transpose()<<" pos "<<state_inout.pos_end.transpose()<<" ba "<<state_inout.bias_a.transpose()<<std::endl;
  #endif

  /*** undistort each lidar point (backward propagation) ***/
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    acc_imu<<VEC_FROM_ARRAY(head->acc);
    // std::cout<<"head imu acc: "<<acc_imu.transpose()<<std::endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    angvel_avr<<VEC_FROM_ARRAY(head->gyr);

    int i = 0;
    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      Eigen::Matrix3d R_i(R_imu * Exp(angvel_avr, dt));
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt + R_i * Lidar_offset_to_IMU - pos_liD_e);

      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d P_compensate = state_inout.rot_end.transpose() * (R_i * P_i + T_ei);

      /// save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {std::cout<<"no imu data"<<std::endl;return;};
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_Initial(meas, stat, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT)
    {
      imu_need_init_ = false;
      // std::cout<<"mean acc: "<<mean_acc<<" acc measures in word frame:"<<state.rot_end.transpose()*mean_acc<<std::endl;
      ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
               stat.gravity[0], stat.gravity[1], stat.gravity[2], stat.bias_g[0], stat.bias_g[1], stat.bias_g[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
    }

    return;
  }

  /// Undistort points： the first point is assummed as the base frame
  /// Compensate lidar points with IMU rotation (with only rotation now)
  UndistortPcl(meas, stat, *cur_pcl_un_);

  t2 = omp_get_wtime();

  // {
  //   static ros::Publisher pub_UndistortPcl =
  //       nh.advertise<sensor_msgs::PointCloud2>("/livox_undistort", 100);
  //   sensor_msgs::PointCloud2 pcl_out_msg;
  //   pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
  //   pcl_out_msg.header.stamp = ros::Time().fromSec(meas.lidar_beg_time);
  //   pcl_out_msg.header.frame_id = "/livox";
  //   pub_UndistortPcl.publish(pcl_out_msg);
  // }

  /// Record last measurements
  last_imu_   = meas.imu.back();

  t3 = omp_get_wtime();
  
  std::cout<<"[ IMU Process ]: Time: "<<t3 - t1<<std::endl;
}

// /// *************ROS Node
// /// IMU relative variables
// std::mutex mtx_buffer;
// std::condition_variable sig_buffer;
// pcl::PointCloud<PointType> v_pcl;
// bool b_exit = false;
// bool b_reset = false;
// bool b_first = true;

// /// Buffers for measurements
// double lidar_end_time = 0.0;
// double last_timestamp_lidar = -1;
// double last_timestamp_imu   = -1;

// std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
// std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

// void SigHandle(int sig)
// {
//   b_exit = true;
//   ROS_WARN("catch sig %d", sig);
//   sig_buffer.notify_all();
// }

// void pointcloud_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
// {
//   const double timestamp = msg->header.stamp.toSec();
//   ROS_DEBUG("get point cloud at time: %.6f", timestamp);

//   mtx_buffer.lock();

//   if (timestamp < last_timestamp_lidar)
//   {
//     ROS_ERROR("lidar loop back, clear buffer");
//     lidar_buffer.clear();
//   }
//   last_timestamp_lidar = timestamp;

//   lidar_buffer.push_back(msg);

//   mtx_buffer.unlock();
//   sig_buffer.notify_all();
// }

// void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
// {
//   sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

//   double timestamp = msg->header.stamp.toSec();
//   // ROS_DEBUG("get imu at time: %.6f", timestamp);

//   mtx_buffer.lock();

//   if (timestamp < last_timestamp_imu) {
//     ROS_ERROR("imu loop back, clear buffer");
//     imu_buffer.clear();
//     b_reset = true;
//     b_first = true;
//   }
//   last_timestamp_imu = timestamp;

//   imu_buffer.push_back(msg);

//   mtx_buffer.unlock();
//   sig_buffer.notify_all();
// }

// bool SyncMeasure(MeasureGroup &measgroup, StatesGroup& state_inout) 
// {
//   if (lidar_buffer.empty() || imu_buffer.empty() || (pose_buffer.empty() && (!b_first))) {
//     /// Note: this will happen
//     // ROS_INFO("NO IMU DATA");
//     return false;
//   }

//   /// Add lidar data once when lidar msg comes, and pop from buffer
//   if(v_pcl.empty())
//   {
//     measgroup.lidar = lidar_buffer.front();
//     pcl::fromROSMsg(*(measgroup.lidar), v_pcl);
//     lidar_end_time = measgroup.lidar->header.stamp.toSec() + v_pcl.points.back().curvature / double(1000);
//   }

//   double imu_end_time = imu_buffer.back()->header.stamp.toSec();

//   if (imu_end_time < lidar_end_time)
//   {
//     std::cout<<"[IMU DELAY]: "<<imu_buffer.back()->header.stamp.toSec()<<" "<<lidar_end_time<<std::endl;
//     return false;
//   }

//   /// Add imu data, and pop from buffer
//   measgroup.imu.clear();

//   while (imu_buffer.front()->header.stamp.toSec() < lidar_end_time)
//   {
//     measgroup.imu.push_back(imu_buffer.front());
//     imu_buffer.pop_front();
//   }

//   if(!pose_buffer.empty())
//   {
//     state_inout = pose_buffer.back();
//     pose_buffer.pop_front();
//   }

//   lidar_buffer.pop_front();
//   v_pcl.clear();
  
//   return true;
// }

// void ProcessLoop(std::shared_ptr<ImuProcess> p_imu)
// {
//   ROS_INFO("Start ProcessLoop");

//   ros::Rate r(200);

//   while (ros::ok())
//   {
//     MeasureGroup meas;
//     StatesGroup state_inout;
//     std::unique_lock<std::mutex> lk(mtx_buffer);
//     // ROS_INFO("wait imu");
//     sig_buffer.wait(lk,
//                     [&meas, &state_inout]() -> bool { return SyncMeasure(meas, state_inout) || b_exit; });
//     lk.unlock();

//     if (b_exit) {
//       ROS_INFO("b_exit=true, exit");
//       break;
//     }

//     if (b_reset) {
//       ROS_WARN("reset when rosbag play back");
//       p_imu->Reset();
//       b_reset = false;
//       continue;
//     }
//     p_imu->Process(meas, state_inout);

//     r.sleep();
//   }
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "data_process");
//   ros::NodeHandle nh;
//   signal(SIGINT, SigHandle);

//   ros::Subscriber sub_pcl = nh.subscribe("/laser_cloud_flat", 100, pointcloud_cbk);
//   ros::Subscriber sub_imu = nh.subscribe("/livox/imu", 100, imu_cbk);
//   ros::Subscriber sub_pose = nh.subscribe("/States_updated", 10, pose_cbk);
//   std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

//   /// for debug
//   p_imu->nh = nh;

//   std::thread th_proc(ProcessLoop, p_imu);

//   // ros::spin();
//   ros::Rate r(1000);
//   while (ros::ok()) {
//     if (b_exit) break;

//     ros::spinOnce();
//     r.sleep();
//   }

//   ROS_INFO("Wait for process loop exit");
//   if (th_proc.joinable()) th_proc.join();

//   return 0;
// }
