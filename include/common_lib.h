#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <livox_loam_kp/KeyPointPose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define G_m_s2 (9.8099)  // Gravaty const in GuangDong/China
#define PI_M (3.14159265358)

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
#define MAT_FROM_ARRAY(v) v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define VEC_FROM_ARRAY(v) v[0],v[1],v[2]
#define CORRECR_PI(v)     ((v > 1.57) ? (v - PI_M) : ((v < -1.57) ? (v + PI_M) : v))

Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
Eigen::Vector3d Zero3d(0, 0, 0);

typedef livox_loam_kp::KeyPointPoseConstPtr KPPoseConstPtr;
typedef livox_loam_kp::Pose6D Pose6D;
typedef geometry_msgs::Vector3 Vec3;

auto set_pose6d(const double t, const Eigen::Vector3d &a, const Eigen::Vector3d &g, \
                  const Eigen::Vector3d &b_a, const Eigen::Vector3d &b_g, \
                  const Eigen::Vector3d &v, const Eigen::Vector3d &p, const Eigen::Matrix3d &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.bias_acc[i] = b_a(i);
        rot_kp.bias_gyr[i] = b_g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    return rot_kp;
}

// auto set_pose6d(const float t, const Eigen::Vector3f &a, const Eigen::Vector3f &g, \
//                   const Eigen::Vector3f &b_a, const Eigen::Vector3f &b_g, \
//                   const Eigen::Vector3f &v, const Eigen::Vector3f &p, const Eigen::Matrix3d &R)
// {
//     Pose6D rot_kp;
//     rot_kp.offset_time = t;

//     tf::vectorEigenToMsg(a, rot_kp.acc);
//     tf::vectorEigenToMsg(g, rot_kp.gyr);
//     tf::vectorEigenToMsg(b_a, rot_kp.bias_acc);
//     tf::vectorEigenToMsg(b_g, rot_kp.bias_gyr);
//     tf::vectorEigenToMsg(v, rot_kp.vel);
//     tf::vectorEigenToMsg(p, rot_kp.pos);

//     for (int i = 0; i < 3; i++)
//     {
//         rot_kp.acc[i] = a(i);
//         rot_kp.gyr[i] = g(i);
//         rot_kp.bias_acc[i] = b_a(i);
//         rot_kp.bias_gyr[i] = b_g(i);
//         rot_kp.vel[i] = v(i);
//         rot_kp.pos[i] = p(i);
//         for (int j = 0; j < 3; j++)  rot_kp.rot[i] = R(i, j);
//     }
//     return rot_kp;
// }

template<typename T>
auto correct_pi(const T v) {return CORRECR_PI(v);}

template<typename T>
auto correct_pi(const Eigen::Matrix<T, 3, 1> v)
{
    Eigen::Matrix<T, 3, 1> g;
    for(int i=0;i<3;i++)
    {
        g[i] = CORRECR_PI(T(v[i]));
    }
    return g;
}

#endif
