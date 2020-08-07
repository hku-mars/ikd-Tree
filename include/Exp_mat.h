#ifndef EXP_MAT_H
#define EXP_MAT_H

#include <math.h>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <common_lib.h>

template<typename T>
auto Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const T &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eigen::Matrix<T, 3, 3>(Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K);
    }
    else
    {
        return Eye3;
    }
}

template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    
    T norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.0000001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

// template<typename T>
// cv::Mat Exp(const T &v1, const T &v2, const T &v3)
// {
    
//     T norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
//     cv::Mat Eye3 = cv::Mat::eye(3, 3, CV_32F);
//     if (norm > 0.0000001)
//     {
//         T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
//         cv::Mat K = (cv::Mat_<T>(3,3) << SKEW_SYM_MATRX(r_ang));

//         /// Roderigous Tranformation
//         return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
//     }
//     else
//     {
//         return Eye3;
//     }
// }

#endif
