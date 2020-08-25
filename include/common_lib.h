#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <livox_loam_kp/KeyPointPose.h>
// #include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


#define G_m_s2 (9.8099)         // Gravaty const in GuangDong/China
#define DIM_OF_STATES (18)      // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_OF_PROC_N (12)      // Dimension of process noise (Let Dim(SO(3)) = 3)
#define PI_M (3.14159265358)

#define DIM_OF_STATES_SQUARE (18*18)

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
#define MAT_FROM_ARRAY(v) v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define VEC_FROM_ARRAY(v) v[0],v[1],v[2]
#define CORRECR_PI(v)     ((v > 1.57) ? (v - PI_M) : ((v < -1.57) ? (v + PI_M) : v))
#define ARRAY_FROM_EIGEN(mat)  mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)  std::vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())

Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
Eigen::Matrix3f Eye3f(Eigen::Matrix3f::Identity());
Eigen::Vector3d Zero3d(0, 0, 0);
Eigen::Vector3f Zero3f(0, 0, 0);

typedef livox_loam_kp::KeyPointPoseConstPtr KPPoseConstPtr;
typedef livox_loam_kp::KeyPointPose KPPose;
typedef livox_loam_kp::Pose6D Pose6D;
typedef geometry_msgs::Vector3 Vec3;

template<typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g, \
                const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    // Eigen::Map<Eigen::Matrix3d>(rot_kp.rot, 3,3) = R;
    return std::move(rot_kp);
}

template<typename T>
void save_states(KPPose &states, const std_msgs::Header &header, \
                const Eigen::Matrix<T, 3, 1> &gravity, const Eigen::Matrix<T, 3, 1> &bg, \
                const Eigen::Matrix<T, 3, 1> &ba, const Eigen::Matrix<T, 3, 1> &p, \
                const Eigen::Matrix<T, 3, 1> &v,  const Eigen::Matrix<T, 3, 3> &R, \
                const Eigen::Matrix<T, DIM_OF_STATES, DIM_OF_STATES> &cov)
{
    states.header   = header;
    states.gravity  = std::vector<T> (ARRAY_FROM_EIGEN(gravity));
    states.bias_gyr = std::vector<T> (ARRAY_FROM_EIGEN(bg));
    states.bias_acc = std::vector<T> (ARRAY_FROM_EIGEN(ba));
    states.pos_end  = std::vector<T> (ARRAY_FROM_EIGEN(p));
    states.vel_end  = std::vector<T> (ARRAY_FROM_EIGEN(v));
    states.rot_end  = std::vector<T> (ARRAY_FROM_EIGEN(R));
    states.cov      = std::vector<T> (ARRAY_FROM_EIGEN(cov));
}

template<typename T>
void save_states(KPPose &states, const std_msgs::Header &header, \
                const Eigen::Matrix<T, DIM_OF_STATES, 1> &state_vec, \
                const Eigen::Matrix<T, DIM_OF_STATES, DIM_OF_STATES> &cov)
{
    states.header   = header;
    states.gravity  = std::vector<T> (state_vec.data(), state_vec.data() + 3);
    states.bias_gyr = std::vector<T> (state_vec.data() + 3, state_vec.data() + 6);
    states.bias_acc = std::vector<T> (state_vec.data() + 6, state_vec.data() + 9);
    states.pos_end  = std::vector<T> (state_vec.data() + 9, state_vec.data() + 12);
    states.vel_end  = std::vector<T> (state_vec.data() + 12, state_vec.data() + 15);
    states.rot_end  = std::vector<T> (state_vec.data() + 15, state_vec.data() + 18);
    states.cov      = std::vector<T> (ARRAY_FROM_EIGEN(cov));
}

template<typename T>
auto correct_pi(const T &v) {return CORRECR_PI(v);}

template<typename T>
Eigen::Matrix<T, 3, 1> correct_pi(const Eigen::Matrix<T, 3, 1> &v)
{
    Eigen::Matrix<T, 3, 1> g;
    for(int i=0;i<3;i++)
    {
        g[i] = CORRECR_PI(T(v[i]));
    }
    return g;
}

#endif
