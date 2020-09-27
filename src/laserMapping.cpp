// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <math.h>
#include <fstream>
#include <unistd.h>
#include <Python.h>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <common_lib.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fast_lio/KeyPointPose.h>
#include <geometry_msgs/Vector3.h>
#include "Exp_mat.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

#define DEBUG_PRINT

#define INIT_TIME           (2.0)
#define LASER_POINT_COV     (0.0010)
#define NUM_MATCH_POINTS    (5)
#define NUM_MAX_ITERATIONS  (15)
#define LASER_FRAME_INTEVAL (0.1)
#define STR2(a)             #a

typedef pcl::PointXYZI PointType;

std::string root_dir = ROOT_DIR;

int iterCount = 0;

float timeLaserCloudCornerLast = 0;
float timeLaserCloudSurfLast   = 0;
float timeLaserCloudFullRes    = 0;
double timeIMUkpLast = 0;
double timeIMUkpCur  = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast   = false;
bool newLaserCloudFullRes    = false;

int laserCloudCenWidth  = 10;
int laserCloudCenHeight = 5;
int laserCloudCenDepth  = 10;
const int laserCloudWidth  = 21;
const int laserCloudHeight = 11;
const int laserCloudDepth  = 21;

const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;//4851
int count_effect_point  = 0;

int laserCloudValidInd[125];

int laserCloudSurroundInd[125];

//corner feature
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_down(new pcl::PointCloud<PointType>());
//surf feature
std::deque<sensor_msgs::PointCloud2> LaserCloudSurfaceBuff;
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurf_down(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());

// pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
// pcl::PointCloud<PointType>::Ptr laserCloudSurround_corner(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurround2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround2_corner(new pcl::PointCloud<PointType>());
//corner feature in map
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
//surf feature in map
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());

std::vector< Eigen::Matrix<float,7,1> > keyframe_pose;
std::vector< Eigen::Matrix4f > pose_map;
std::deque< fast_lio::KeyPointPose > rot_kp_imu_buff;
//all points
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes2(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());


pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];

pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];

pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];

pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

//optimization states 
double transformTobeMapped[6] = {0};
//optimization states after mapping
double transformAftMapped[6] = {0};
//last optimization states
double transformLastMapped[6] = {0};

//estimated rotation and translation;
Eigen::Matrix3d R_global_cur(Eigen::Matrix3d::Identity());
Eigen::Matrix3d R_global_last(Eigen::Matrix3d::Identity());
Eigen::Vector3d T_global_cur(0, 0, 0);
Eigen::Vector3d T_global_last(0, 0, 0);
Eigen::Vector3d V_global_cur(0, 0, 0);
Eigen::Vector3d V_global_last(0, 0, 0);
// Eigen::MatrixXf cov_stat_cur(Eigen::Matrix<float, DIM_OF_STATES, DIM_OF_STATES>::Zero());

//final iteration resdual
double deltaR = 0.0;
double deltaT = 0.0;

double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}
double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

void transformUpdate()
{
    for (int i = 0; i < 6; i++) {
        transformLastMapped[i] = transformAftMapped[i];
        transformAftMapped[i] = transformTobeMapped[i];
    }
}
//lidar coordinate sys to world coordinate sys
void pointAssociateToMap(PointType const * const pi, PointType * const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(R_global_cur * p_body + T_global_cur);
    
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointAssociateToMap(PointType const * const pi, pcl::PointXYZRGB * const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(R_global_cur * p_body + T_global_cur);
    
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    //po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity*10000;

    //std::cout<<"DEBUG reflection_map "<<reflection_map<<std::endl;

    if (reflection_map < 30)
    {
        int green = (reflection_map * 255 / 30);
        po->r = 0;
        po->g = green & 0xff;
        po->b = 0xff;
    }
    else if (reflection_map < 90)
    {
        int blue = (((90 - reflection_map) * 255) / 60);
        po->r = 0x0;
        po->g = 0xff;
        po->b = blue & 0xff;
    }
    else if (reflection_map < 150)
    {
        int red = ((reflection_map-90) * 255 / 60);
        po->r = red & 0xff;
        po->g = 0xff;
        po->b = 0x0;
    }
    else
    {
        int green = (((255-reflection_map) * 255) / (255-150));
        po->r = 0xff;
        po->g = green & 0xff;
        po->b = 0;
    }
}

#ifdef USING_CORNER
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
    timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

    laserCloudCornerLast->clear();
    pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

    newLaserCloudCornerLast = true;
}
#endif

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
    LaserCloudSurfaceBuff.push_back(*laserCloudSurfLast2);

    timeLaserCloudSurfLast = LaserCloudSurfaceBuff.front().header.stamp.toSec();

    newLaserCloudSurfLast = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
    laserCloudFullRes->clear();
    laserCloudFullResColor->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

    newLaserCloudFullRes = true;
}

void KeyPointPose6DHandler(const fast_lio::KeyPointPoseConstPtr& KeyPointPose)
{
    rot_kp_imu_buff.push_back(*KeyPointPose);

    timeIMUkpCur = rot_kp_imu_buff.front().header.stamp.toSec();
}

bool sync_packages()
{
    if(rot_kp_imu_buff.size() != LaserCloudSurfaceBuff.size())
    {
        return false;
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

#ifdef USING_CORNER
    ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_sharp", 100, laserCloudCornerLastHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
            ("/livox_cloud", 100, laserCloudFullResHandler);
#else
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
            ("/livox_undistort", 100, laserCloudFullResHandler);
#endif

    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
            ("/livox_undistort", 100, laserCloudSurfLastHandler);
    ros::Subscriber KeyPointPose6D = nh.subscribe<fast_lio::KeyPointPose>
            ("/Pose6D_IMUKeyPoints", 100, KeyPointPose6DHandler);

    // ros::Subscriber subIMUOri = nh.subscribe<sensor_msgs::>

    ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_surround", 100);
    ros::Publisher pubLaserCloudSurround_corner = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_surround_corner", 100);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100);
    ros::Publisher pubSolvedPose6D = nh.advertise<fast_lio::KeyPointPose>
            ("/Pose6D_Solved", 100);
    
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 10);
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/camera_init";
    odomAftMapped.child_frame_id = "/aft_mapped";
    fast_lio::KeyPointPose Pose6D_Solved;

    std::string map_file_path;
    bool dense_map_en;
    double filter_size_corner_min, filter_size_surf_min, filter_size_map_min;
    ros::param::get("~dense_map_enable",dense_map_en);
    ros::param::get("~map_file_path",map_file_path);
    ros::param::get("~filter_size_corner",filter_size_corner_min);
    ros::param::get("~filter_size_surf",filter_size_surf_min);
    ros::param::get("~filter_size_map",filter_size_map_min);

    PointType pointOri, pointSel, coeff;

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

    bool isDegenerate = false;
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
    //VoxelGrid
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    downSizeFilterCorner.setLeafSize(filter_size_corner_min, filter_size_corner_min, filter_size_corner_min);
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    for (int i = 0; i < laserCloudNum; i++)
    {
        laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
    }
    
    std::vector<double> T1, s_plot, s_plot2, s_plot3;
    double aver_time_consu = 0;
    double frame_num = 0;

//------------------------------------------------------------------------------------------------------
    std::ofstream fout_pre, fout_out;
    
    fout_pre.open(FILE_DIR("mat_pre.txt"),std::ios::out);
    fout_out.open(FILE_DIR("mat_out.txt"),std::ios::out);
    if (fout_pre && fout_out)
        std::cout << "~~~~"<<ROOT_DIR<<" file opened" << std::endl;
    else
        std::cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << std::endl;
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();

        while(!LaserCloudSurfaceBuff.empty() && !rot_kp_imu_buff.empty() && sync_packages()) 
        {
            static int degenerate_count = 0;
            static double first_lidar_time = LaserCloudSurfaceBuff.front().header.stamp.toSec();
            bool Need_Init = ((timeLaserCloudSurfLast - first_lidar_time) < INIT_TIME) ? true : false;
            if(Need_Init) {std::cout<<"||||||||||Initiallizing LiDar||||||||||"<<std::endl;}
            std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~frame buff size: "<<LaserCloudSurfaceBuff.size()<<" time of this frame: "<<timeLaserCloudSurfLast<<std::endl;
            double t1,t2,t3,t4;
            double match_start, match_time, solve_start, solve_time, pca_time, svd_time;

            match_time = 0;
            solve_time = 0;
            pca_time   = 0;
            svd_time   = 0;

            t1 = omp_get_wtime();

            newLaserCloudCornerLast = false;
            newLaserCloudSurfLast = false;
            newLaserCloudFullRes = false;

            //transformAssociateToMap();
            std::cout<<"DEBUG mapping start "<<std::endl;

            PointType pointOnYAxis;
            pointOnYAxis.x = 0.0;
            pointOnYAxis.y = 10.0;
            pointOnYAxis.z = 0.0;
            
            /** Get the rotations and translations of IMU keypoints in a frame **/
            Eigen::Vector3d gravity, bias_g, bias_a;
            Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov_stat_cur;

            gravity<<VEC_FROM_ARRAY(rot_kp_imu_buff.front().gravity);
            bias_g<<VEC_FROM_ARRAY(rot_kp_imu_buff.front().bias_gyr);
            bias_a<<VEC_FROM_ARRAY(rot_kp_imu_buff.front().bias_acc);
            T_global_cur<<VEC_FROM_ARRAY(rot_kp_imu_buff.front().pos_end);
            V_global_cur<<VEC_FROM_ARRAY(rot_kp_imu_buff.front().vel_end);
            R_global_cur=Eigen::Map<Eigen::Matrix3d>(rot_kp_imu_buff.front().rot_end.data());
            cov_stat_cur=Eigen::Map<Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> >(rot_kp_imu_buff.front().cov.data());

            Eigen::Vector3d euler_cur = correct_pi(R_global_cur.eulerAngles(1, 0, 2));

            transformTobeMapped[0]  = euler_cur(0);
            transformTobeMapped[1]  = euler_cur(1);
            transformTobeMapped[2]  = euler_cur(2);
            transformTobeMapped[3]  = T_global_cur(0);
            transformTobeMapped[4]  = T_global_cur(1);
            transformTobeMapped[5]  = T_global_cur(2);

            fout_pre << std::setw(10) << timeLaserCloudSurfLast << " " << euler_cur.transpose()*57.3 << " " << T_global_cur.transpose() << " " << V_global_cur.transpose() << std::endl;

            #ifdef DEBUG_PRINT
            std::cout<<"pre-integrated states: "<<euler_cur.transpose()*57.3<<" "<<T_global_cur.transpose()<<" "<<V_global_cur.transpose()<<" "<<bias_g.transpose()<<" "<<bias_a.transpose()<<std::endl;
            #endif
            
            pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

            int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
            int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
            int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;

            if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
            if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
            if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;

            while (centerCubeI < 3)
            {
                for (int j = 0; j < laserCloudHeight; j++)
                {
                    for (int k = 0; k < laserCloudDepth; k++)
                    {
                        int i = laserCloudWidth - 1;

                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; i >= 1; i--) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i - 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        }

                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }
                centerCubeI++;
                laserCloudCenWidth++;
            }

            while (centerCubeI >= laserCloudWidth - 3) {
                for (int j = 0; j < laserCloudHeight; j++) {
                    for (int k = 0; k < laserCloudDepth; k++) {
                        int i = 0;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; i < laserCloudWidth - 1; i++)
                        {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        }

                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeI--;
                laserCloudCenWidth--;
            }

            while (centerCubeJ < 3) {
                for (int i = 0; i < laserCloudWidth; i++) {
                    for (int k = 0; k < laserCloudDepth; k++) {
                        int j = laserCloudHeight - 1;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; j >= 1; j--) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth*(j - 1) + laserCloudWidth * laserCloudHeight*k];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
                        }
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeJ++;
                laserCloudCenHeight++;
            }

            while (centerCubeJ >= laserCloudHeight - 3) {
                for (int i = 0; i < laserCloudWidth; i++) {
                    for (int k = 0; k < laserCloudDepth; k++) {
                        int j = 0;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; j < laserCloudHeight - 1; j++) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth*(j + 1) + laserCloudWidth * laserCloudHeight*k];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
                        }
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeJ--;
                laserCloudCenHeight--;
            }

            while (centerCubeK < 3) {
                for (int i = 0; i < laserCloudWidth; i++) {
                    for (int j = 0; j < laserCloudHeight; j++) {
                        int k = laserCloudDepth - 1;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

                        for (; k >= 1; k--) {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k - 1)];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
                        }
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeK++;
                laserCloudCenDepth++;
            }

            while (centerCubeK >= laserCloudDepth - 3)
            {
                for (int i = 0; i < laserCloudWidth; i++)
                {
                    for (int j = 0; j < laserCloudHeight; j++)
                    {
                        int k = 0;
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    
                        for (; k < laserCloudDepth - 1; k++)
                        {
                            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k + 1)];
                            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
                        }

                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeCornerPointer;
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                                laserCloudCubeSurfPointer;
                        laserCloudCubeCornerPointer->clear();
                        laserCloudCubeSurfPointer->clear();
                    }
                }

                centerCubeK--;
                laserCloudCenDepth--;
            }

            int laserCloudValidNum    = 0;
            int laserCloudSurroundNum = 0;

            for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) 
            {
                for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) 
                {
                    for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) 
                    {
                        if (i >= 0 && i < laserCloudWidth &&
                                j >= 0 && j < laserCloudHeight &&
                                k >= 0 && k < laserCloudDepth) 
                        {

                            float centerX = 50.0 * (i - laserCloudCenWidth);
                            float centerY = 50.0 * (j - laserCloudCenHeight);
                            float centerZ = 50.0 * (k - laserCloudCenDepth);

                            bool isInLaserFOV = false;
                            for (int ii = -1; ii <= 1; ii += 2) 
                            {
                                for (int jj = -1; jj <= 1; jj += 2) 
                                {
                                    for (int kk = -1; kk <= 1; kk += 2) 
                                    {

                                        float cornerX = centerX + 25.0 * ii;
                                        float cornerY = centerY + 25.0 * jj;
                                        float cornerZ = centerZ + 25.0 * kk;

                                        float squaredSide1 = (transformTobeMapped[3] - cornerX)
                                                * (transformTobeMapped[3] - cornerX)
                                                + (transformTobeMapped[4] - cornerY)
                                                * (transformTobeMapped[4] - cornerY)
                                                + (transformTobeMapped[5] - cornerZ)
                                                * (transformTobeMapped[5] - cornerZ);

                                        float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
                                                + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                                + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                                        float check1 = 100.0 + squaredSide1 - squaredSide2
                                                - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                                        float check2 = 100.0 + squaredSide1 - squaredSide2
                                                + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                                        if (check1 < 0 && check2 > 0) {
                                            isInLaserFOV = true;
                                        }
                                    }
                                }
                            }

                            if (isInLaserFOV)
                            {
                                laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j
                                        + laserCloudWidth * laserCloudHeight * k;
                                laserCloudValidNum ++;
                            }
                            laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j
                                    + laserCloudWidth * laserCloudHeight * k;
                            laserCloudSurroundNum ++;
                        }
                    }
                }
            }

            laserCloudCornerFromMap->clear();
            laserCloudSurfFromMap->clear();
            
            for (int i = 0; i < laserCloudValidNum; i++)
            {
                #ifdef USING_CORNER
                *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
                #endif
                *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
            }

            laserCloudSurfLast->clear();
            laserCloudSurf_down->clear();
            pcl::fromROSMsg(LaserCloudSurfaceBuff.front(), *laserCloudSurfLast);
            /* float cent_point[3] = {0.0};
            int num = 0;
            for (int i = 0; i < laserCloudSurfLast->points.size(); i += 100)
            {
                num ++;
                // std::cout<<"cent_point[0] "<<cent_point[0]<<cent_point[1]<<cent_point[2];
                // std::cout<<" point: "<<laserCloudSurfLast->points[i]<<std::endl;
                cent_point[0] += (laserCloudSurfLast->points[i].x - cent_point[0]) / num;
                cent_point[1] += (laserCloudSurfLast->points[i].y - cent_point[1]) / num;
                cent_point[2] += (laserCloudSurfLast->points[i].z - cent_point[2]) / num;                
            }

            float center_dist = std::sqrt(cent_point[0] * cent_point[0] + cent_point[1] * cent_point[1] 
                                            + cent_point[2] * cent_point[2]);
            double filter_size_surf, filter_size_map;
            filter_size_surf = double(center_dist - 5.0) / 100.0 * (0.5 - filter_size_surf_min) + filter_size_surf_min;
            filter_size_surf = CONSTRAIN(filter_size_surf, filter_size_surf_min, 0.4);
            filter_size_map  = filter_size_surf * 1.3333;
            downSizeFilterSurf.setLeafSize(filter_size_surf, filter_size_surf, filter_size_surf);
            downSizeFilterMap.setLeafSize(filter_size_map, filter_size_map, filter_size_map);
            std::cout<<"center distance: "<<center_dist<<" surf size: "<<filter_size_surf<<" map size: "<<filter_size_map<<std::endl; */

            downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
            downSizeFilterSurf.filter(*laserCloudSurf_down);

            downSizeFilterMap.setInputCloud(laserCloudSurfFromMap);
            downSizeFilterMap.filter(*laserCloudSurfFromMap);

#ifdef USING_CORNER
            laserCloudCornerLast_down->clear();
            downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
            downSizeFilterCorner.filter(*laserCloudCornerLast_down);
            int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
            std::cout<<"DEBUG MAPPING CornerLast_down : "<<laserCloudCornerLast_down->points.size()<<" corner full: "
            <<laserCloudCornerLast->points.size()<<std::endl;
#endif
            int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();
            int laserCloudSurf_down_size = laserCloudSurf_down->points.size();

            std::cout<<"DEBUG MAPPING laserCloudSurf_down : "<<laserCloudSurf_down_size<<" laserCloudSurf : "
            <<laserCloudSurfLast->points.size()<<std::endl;
            std::cout<<"DEBUG MAPPING laserCloudValidNum : "<<laserCloudValidNum<<" laserCloudSurfFromMapNum : "
            <<laserCloudSurfFromMapNum<<std::endl;

            

            pcl::PointCloud<PointType>::Ptr coeffSel_tmpt
                (new pcl::PointCloud<PointType>(*laserCloudSurf_down));
            pcl::PointCloud<PointType>::Ptr laserCloudSurf_down_updated
                (new pcl::PointCloud<PointType>(*laserCloudSurf_down));

            t2 = omp_get_wtime();

#ifdef USING_CORNER
            if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
            {
                match_start = omp_get_wtime();
                kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
#else
            if (laserCloudSurfFromMapNum > 100)
            {  
                match_start = omp_get_wtime();
#endif
                pcl::KdTreeFLANN<PointType> kdtreeSurfFromMap_tmpt;
                kdtreeSurfFromMap_tmpt.setInputCloud(laserCloudSurfFromMap);
                std::vector<int> pointSearchInd_corner;
                std::vector<float> pointSearchSqDis_corner;
                std::vector<bool> point_selected_surf(laserCloudSurf_down_size, true);
                std::vector<std::vector<int>> pointSearchInd_surf(laserCloudSurf_down_size);

                int  rematch_num = 0;
                bool rematch_en = 0;
                
                for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) 
                {
                    laserCloudOri->clear();
                    coeffSel->clear();

#ifdef USING_CORNER
                    for (int i = 0; i < laserCloudCornerLast->points.size(); i++) {
                        pointOri = laserCloudCornerLast->points[i];

                        pointAssociateToMap(&pointOri, &pointSel);
                        //find the closest 5 points
                        if (iterCount == 0 || rematch_en)
                        {
                            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd_corner, pointSearchSqDis_corner);
                        }

                        if (pointSearchSqDis_corner[4] < 1.5) {
                            float cx = 0;
                            float cy = 0;
                            float cz = 0;
                            for (int j = 0; j < 5; j++) {
                                cx += laserCloudCornerFromMap->points[pointSearchInd_corner[j]].x;
                                cy += laserCloudCornerFromMap->points[pointSearchInd_corner[j]].y;
                                cz += laserCloudCornerFromMap->points[pointSearchInd_corner[j]].z;
                            }
                            cx /= 5;
                            cy /= 5;
                            cz /= 5;
                            //mean square error
                            float a11 = 0;
                            float a12 = 0;
                            float a13 = 0;
                            float a22 = 0;
                            float a23 = 0;
                            float a33 = 0;
                            for (int j = 0; j < 5; j++) {
                                float ax = laserCloudCornerFromMap->points[pointSearchInd_corner[j]].x - cx;
                                float ay = laserCloudCornerFromMap->points[pointSearchInd_corner[j]].y - cy;
                                float az = laserCloudCornerFromMap->points[pointSearchInd_corner[j]].z - cz;

                                a11 += ax * ax;
                                a12 += ax * ay;
                                a13 += ax * az;
                                a22 += ay * ay;
                                a23 += ay * az;
                                a33 += az * az;
                            }
                            a11 /= 5;
                            a12 /= 5;
                            a13 /= 5;
                            a22 /= 5;
                            a23 /= 5;
                            a33 /= 5;

                            matA1.at<float>(0, 0) = a11;
                            matA1.at<float>(0, 1) = a12;
                            matA1.at<float>(0, 2) = a13;
                            matA1.at<float>(1, 0) = a12;
                            matA1.at<float>(1, 1) = a22;
                            matA1.at<float>(1, 2) = a23;
                            matA1.at<float>(2, 0) = a13;
                            matA1.at<float>(2, 1) = a23;
                            matA1.at<float>(2, 2) = a33;

                            cv::eigen(matA1, matD1, matV1);

                            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                                float x0 = pointSel.x;
                                float y0 = pointSel.y;
                                float z0 = pointSel.z;
                                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                                //OA = (x0 - x1, y0 - y1, z0 - z1),OB = (x0 - x2, y0 - y2, z0 - z2)，AB = （x1 - x2, y1 - y2, z1 - z2）
                                //cross:
                                //|  i      j      k  |
                                //|x0-x1  y0-y1  z0-z1|
                                //|x0-x2  y0-y2  z0-z2|
                                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                    * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                    * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                                    * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                                - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                                + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                                float ld2 = a012 / l12;
                                //if(fabs(ld2) > 1) continue;

                                float s = 1 - 0.9 * fabs(ld2);

                                coeff.x = s * la;
                                coeff.y = s * lb;
                                coeff.z = s * lc;
                                coeff.intensity = s * ld2;

                                if (s > 0.1) {
                                    laserCloudOri->push_back(pointOri);
                                    coeffSel->push_back(coeff);
                                }
                            }
                        }
                    }
#endif

                    omp_set_num_threads(4);
                    #pragma omp parallel for
                    for (int i = 0; i < laserCloudSurf_down_size; i++)
                    {
                        PointType &pointOri_tmpt = laserCloudSurf_down->points[i];
                        PointType &pointSel_tmpt = laserCloudSurf_down_updated->points[i];
                        pointAssociateToMap(&pointOri_tmpt, &pointSel_tmpt);

                        std::vector<float> pointSearchSqDis_surf;
                        auto &points_near = pointSearchInd_surf[i];
                        if (iterCount == 0 || rematch_en)
                        {
                            /** Rematch (Reprojection) **/
                            kdtreeSurfFromMap_tmpt.nearestKSearch(pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);
                            if (pointSearchSqDis_surf[NUM_MATCH_POINTS - 1] < 5.0)
                            {
                                point_selected_surf[i] = true;
                            }
                        }
                        
                        if (! point_selected_surf[i]) continue;

                        double pca_start = omp_get_wtime();

                        /// using minimum square method
                        cv::Mat matA0(NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all(0));
                        cv::Mat matB0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(-1));
                        cv::Mat matX0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(0));

                        for (int j = 0; j < NUM_MATCH_POINTS; j++)
                        {
                            matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[points_near[j]].x;
                            matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[points_near[j]].y;
                            matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[points_near[j]].z;
                        }

                        //matA0*matX0=matB0
                        //AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
                        //(X,Y,Z)<=>mat_a0
                        //A/D, B/D, C/D <=> mat_x0
            
                        cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);  //TODO

                        float pa = matX0.at<float>(0, 0);
                        float pb = matX0.at<float>(1, 0);
                        float pc = matX0.at<float>(2, 0);
                        float pd = 1;

                        //ps is the norm of the plane normal vector
                        //pd is the distance from point to plane
                        float ps = sqrt(pa * pa + pb * pb + pc * pc);
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;

                        bool planeValid = true;
                        for (int j = 0; j < NUM_MATCH_POINTS; j++)
                        {
                            if (fabs(pa * laserCloudSurfFromMap->points[points_near[j]].x +
                                        pb * laserCloudSurfFromMap->points[points_near[j]].y +
                                        pc * laserCloudSurfFromMap->points[points_near[j]].z + pd) > 0.1)
                            {
                                planeValid = false;
                                point_selected_surf[i] = false;
                                break;
                            }
                        }

                        if (planeValid) 
                        {
                            //loss fuction
                            float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
                            //if(fabs(pd2) > 0.1) continue;
                            float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y + pointSel_tmpt.z * pointSel_tmpt.z));

                            if (s > 0.1)
                            {
                                point_selected_surf[i] = true;
                                coeffSel_tmpt->points[i].x = s * pa;
                                coeffSel_tmpt->points[i].y = s * pb;
                                coeffSel_tmpt->points[i].z = s * pc;
                                coeffSel_tmpt->points[i].intensity = s * pd2;
                            }
                            else
                            {
                                point_selected_surf[i] = false;
                            }
                        }

                        pca_time += omp_get_wtime() - pca_start;
                    }

                    double total_residual = 0.0;
                    for (int i = 0; i < coeffSel_tmpt->points.size(); i++)
                    {
                        float error_abs = std::abs(coeffSel_tmpt->points[i].intensity);
                        if (point_selected_surf[i]  && (error_abs < 0.5))
                        {
                            laserCloudOri->push_back(laserCloudSurf_down->points[i]);
                            coeffSel->push_back(coeffSel_tmpt->points[i]);
                            total_residual += error_abs;
                        }
                    }

                    int laserCloudSelNum = laserCloudOri->points.size();
                    double ave_residual = total_residual / laserCloudSelNum;

                    if(iterCount == 1) std::cout << "DEBUG mapping select all points : " << coeffSel->size() << "  " << count_effect_point << std::endl;

                    count_effect_point = 0;

                    match_time += omp_get_wtime() - match_start;
                    match_start = omp_get_wtime();
                    solve_start = omp_get_wtime();

                    if (laserCloudSelNum < 50) {
                        continue;
                    }

                    Eigen::MatrixXd H(laserCloudSelNum, DIM_OF_STATES) ;
                    Eigen::VectorXd res(laserCloudSelNum);

                    omp_set_num_threads(4);
                    #pragma omp parallel for
                    for (int i = 0; i < laserCloudSelNum; i++)
                    {
                        const PointType &laser_p = laserCloudOri->points[i];
                        const PointType &nor_vec = coeffSel->points[i];
                        
                        Eigen::Vector3d point_this(laser_p.x, laser_p.y, laser_p.z);
                        Eigen::Vector3d vect_norm(nor_vec.x, nor_vec.y, nor_vec.z);
                        Eigen::Matrix3d point_crossmat;
                        point_crossmat<<SKEW_SYM_MATRX(point_this);
                        
                        Eigen::Vector3d A(point_crossmat * R_global_cur.transpose() * vect_norm);

                        H.row(i) = Eigen::Matrix<double, 1, DIM_OF_STATES>::Zero();
                        H.block<1,6>(i,0) << VEC_FROM_ARRAY(A), nor_vec.x, nor_vec.y, nor_vec.z;
                        res(i) = - nor_vec.intensity;
                    }

                    Eigen::Vector3d rot_add, t_add, v_add, bg_add, ba_add, g_add;
                    Eigen::VectorXd solution(DIM_OF_STATES);
                    Eigen::MatrixXd K(DIM_OF_STATES, DIM_OF_STATES);

                    if (Need_Init)
                    {
                        /*** Iterative Kalan Filter (initiallization) ***/
                        Eigen::MatrixXd H_init(Eigen::Matrix<double, 9, DIM_OF_STATES>::Zero());
                        Eigen::MatrixXd z_init(Eigen::Matrix<double, 9, 1>::Zero());
                        H_init.block<3,3>(0,0)  = Eigen::Matrix3d::Identity();
                        H_init.block<3,3>(3,3)  = Eigen::Matrix3d::Identity();
                        H_init.block<3,3>(6,15) = Eigen::Matrix3d::Identity();
                        z_init.block<3,1>(0,0)  = - Log(R_global_cur);
                        z_init.block<3,1>(0,0)  = - T_global_cur;

                        auto H_init_T = H_init.transpose();
                        auto &&K_init = cov_stat_cur * H_init_T * (H_init * cov_stat_cur * H_init_T + 0.0001 * Eigen::Matrix<double,9,9>::Identity()).inverse();
                        solution      = K_init * z_init;
                        for (int ind = 0; ind < 3; ind ++)
                        {
                            rot_add[ind] = solution(ind);
                            t_add[ind]   = solution(ind+3);
                            v_add[ind]   = solution(ind+6);
                            bg_add[ind]  = solution(ind+9);
                            ba_add[ind]  = solution(ind+12);
                            g_add[ind]   = solution(ind+15);
                        }

                        R_global_cur.setIdentity();
                        T_global_cur.setZero();
                        V_global_cur.setZero();
                        bias_g      += bg_add;
                        bias_a      += ba_add;
                        cov_stat_cur = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K_init * H_init) * cov_stat_cur;
                    }
                    else
                    {
                        /*** Iterative Kalman Filter ***/
                        auto &&H_T = H.transpose();
                        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 = (H_T * H + (cov_stat_cur / LASER_POINT_COV).inverse()).inverse();
                        K = K_1 * H_T;
                        solution = K * res;
                        // std::cout<<"***solution: "<<solution.transpose()<<std::endl;
                        for (int ind = 0; ind < 3; ind ++)
                        {
                            rot_add[ind] = solution(ind);
                            t_add[ind]   = solution(ind+3);
                            v_add[ind]   = solution(ind+6);
                            bg_add[ind]  = solution(ind+9);
                            ba_add[ind]  = solution(ind+12);
                            g_add[ind]   = solution(ind+15);
                        }

                        R_global_cur = R_global_cur * Exp(rot_add);
                        T_global_cur = T_global_cur + t_add;
                        V_global_cur = V_global_cur + v_add;
                        bias_g  += bg_add;
                        bias_a  += ba_add;
                        gravity += g_add;

                        deltaR = rot_add.norm() * 57.3;
                        deltaT = t_add.norm() * 100.0;
                    }

                    euler_cur = correct_pi(R_global_cur.eulerAngles(1, 0, 2));

                    #ifdef DEBUG_PRINT
                    std::cout<<"***new stat: "<<euler_cur.transpose()*57.3<<" "<<T_global_cur.transpose()<<"dR & dT: "<<deltaR<<" "<<deltaT<<" bias: "<<bias_a.transpose()<<" G: "<<gravity.transpose()<<" average res: "<<total_residual/laserCloudSelNum<<std::endl;
                    #endif

                    transformTobeMapped[0] = euler_cur(0);
                    transformTobeMapped[1] = euler_cur(1);
                    transformTobeMapped[2] = euler_cur(2);
                    transformTobeMapped[3] = T_global_cur(0);
                    transformTobeMapped[4] = T_global_cur(1);
                    transformTobeMapped[5] = T_global_cur(2);

                    rematch_en = false;

                    if ((deltaR < 0.01 && deltaT < 0.01))
                    {
                        rematch_en = true;
                        rematch_num ++;
                    }

                    if (rematch_num >= 2)
                    {
                        if (!Need_Init)
                        {
                            /*** Covariance Update ***/
                            cov_stat_cur = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K * H) * cov_stat_cur;
                        }
                        solve_time += omp_get_wtime() - solve_start;
                        break;
                    }
                    solve_time += omp_get_wtime() - solve_start;
                }

                std::cout<<"current time: "<<timeIMUkpCur<<" iteration count: "<<iterCount+1<<std::endl;
                transformUpdate();
            }

            /*** save results ***/
            save_states(Pose6D_Solved, rot_kp_imu_buff.front().header, gravity, bias_g, bias_a, \
                        T_global_cur, V_global_cur, R_global_cur, cov_stat_cur); //std::cout<<"!!!! Sent pose:"<<T_global_cur.transpose()<<std::endl;
            Pose6D_Solved.pose6D.clear();
            pubSolvedPose6D.publish(Pose6D_Solved);

            V_global_last = V_global_cur;
            T_global_last = T_global_cur;
            R_global_last = R_global_cur;
            fout_out << std::setw(10) << timeLaserCloudSurfLast << " " << euler_cur.transpose()*57.3 << " " << T_global_last.transpose() << " " << V_global_last.transpose() << std::endl;
            timeIMUkpLast = timeIMUkpCur;

            LaserCloudSurfaceBuff.pop_front();
            rot_kp_imu_buff.pop_front();
            if (!LaserCloudSurfaceBuff.empty() && !rot_kp_imu_buff.empty())
            {
                timeIMUkpCur  = rot_kp_imu_buff.front().header.stamp.toSec();
                timeLaserCloudSurfLast = LaserCloudSurfaceBuff.front().header.stamp.toSec();
            }

#ifdef USING_CORNER
            for (int i = 0; i < laserCloudCornerLast->points.size(); i++)
            {
                pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);

                int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

                if (pointSel.x + 25.0 < 0) cubeI--;
                if (pointSel.y + 25.0 < 0) cubeJ--;
                if (pointSel.z + 25.0 < 0) cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth)
                {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    laserCloudCornerArray[cubeInd]->push_back(pointSel);
                }
            }
#endif
            
            for (int i = 0; i < laserCloudSurf_down_size; i++)
            {
                PointType &pointSel = laserCloudSurf_down_updated->points[i];

                int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
                int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
                int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

                if (pointSel.x + 25.0 < 0) cubeI--;
                if (pointSel.y + 25.0 < 0) cubeJ--;
                if (pointSel.z + 25.0 < 0) cubeK--;

                if (cubeI >= 0 && cubeI < laserCloudWidth &&
                        cubeJ >= 0 && cubeJ < laserCloudHeight &&
                        cubeK >= 0 && cubeK < laserCloudDepth) {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    laserCloudSurfArray[cubeInd]->push_back(pointSel);
                }
            }

            for (int i = 0; i < laserCloudValidNum; i++)
            {
                int ind = laserCloudValidInd[i];
                laserCloudSurfArray2[ind]->clear();
                downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
                downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

                pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudSurfArray[ind];
                laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
                laserCloudSurfArray2[ind] = laserCloudTemp;

                #ifdef USING_CORNER
                laserCloudCornerArray2[ind]->clear();
                downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
                downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

                laserCloudTemp = laserCloudCornerArray[ind];
                laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
                laserCloudCornerArray2[ind] = laserCloudTemp;
                #endif
            }

            t3 = omp_get_wtime();

            /******* Publish messages:  *******/
            laserCloudSurround2->clear();
            laserCloudSurround2_corner->clear();

            for (int i = 0; i < laserCloudSurroundNum; i++) {
                int ind = laserCloudSurroundInd[i];
                *laserCloudSurround2_corner += *laserCloudCornerArray[ind];
                *laserCloudSurround2 += *laserCloudSurfArray[ind];
            }
            
            laserCloudFullRes2->clear();
            // *laserCloudFullRes2 = *laserCloudFullRes;
            // *laserCloudFullRes2 = dense_map_en ? (*laserCloudSurfLast) : (* laserCloudSurf_down);
            *laserCloudFullRes2 = dense_map_en ? (*laserCloudFullRes) : (* laserCloudSurf_down);

            int laserCloudFullResNum = laserCloudFullRes2->points.size();

            pcl::PointXYZRGB temp_point;

            for (int i = 0; i < laserCloudFullResNum; i++)
            {
                RGBpointAssociateToMap(&laserCloudFullRes2->points[i], &temp_point);
                laserCloudFullResColor->push_back(temp_point);
            }

            sensor_msgs::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            laserCloudFullRes3.header.frame_id = "/camera_init";
            pubLaserCloudFullRes.publish(laserCloudFullRes3);

            sensor_msgs::PointCloud2 laserCloudMap;
            pcl::toROSMsg(*laserCloudSurfFromMap, laserCloudMap);
            laserCloudMap.header.stamp = ros::Time::now();//ros::Time().fromSec(timeLaserCloudCornerLast);
            laserCloudMap.header.frame_id = "/camera_init";
            pubLaserCloudMap.publish(laserCloudMap);

            *laserCloudFullResColor_pcd += *laserCloudFullResColor;

            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                    (transformAftMapped[2], - transformAftMapped[0], - transformAftMapped[1]);

            odomAftMapped.header.stamp = ros::Time::now();//ros::Time().fromSec(timeLaserCloudCornerLast);
            odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
            odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
            odomAftMapped.pose.pose.orientation.z = geoQuat.x;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = transformAftMapped[3];
            odomAftMapped.pose.pose.position.y = transformAftMapped[4];
            odomAftMapped.pose.pose.position.z = transformAftMapped[5];

            pubOdomAftMapped.publish(odomAftMapped);

            static tf::TransformBroadcaster br;
            tf::Transform                   transform;
            tf::Quaternion                  q;
            transform.setOrigin( tf::Vector3( odomAftMapped.pose.pose.position.x,
                                                odomAftMapped.pose.pose.position.y,
                                                odomAftMapped.pose.pose.position.z ) );
            q.setW( odomAftMapped.pose.pose.orientation.w );
            q.setX( odomAftMapped.pose.pose.orientation.x );
            q.setY( odomAftMapped.pose.pose.orientation.y );
            q.setZ( odomAftMapped.pose.pose.orientation.z );
            transform.setRotation( q );
            br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped" ) );

            /*** plot variables ***/
            t4 = omp_get_wtime();
            frame_num ++;
            // aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t1) / frame_num;
            aver_time_consu = aver_time_consu * 0.5 + (t4 - t1) * 0.5;
            T1.push_back(timeLaserCloudSurfLast);
            s_plot.push_back(t4 - t1);
            s_plot2.push_back(double(deltaR));
            s_plot3.push_back(double(deltaT));

            std::cout<<"mapping time : selection "<<t2-t1 <<" match time: "<<match_time<<"  solve time: "<<solve_time<<" total with publish: "<<t4 - t1<<" no publish: "<<t3-t1<<std::endl;
            // std::cout<<"match time: "<<match_time<<"  solve time: "<<solve_time<<std::endl;
        }
        status = ros::ok();
        rate.sleep();
    }
    //--------------------------save map---------------
    std::string surf_filename(map_file_path + "/surf.pcd");
    std::string corner_filename(map_file_path + "/corner.pcd");
    std::string all_points_filename(map_file_path + "/all_points.pcd");
    // std::ofstream keyframe_file(map_file_path + "/key_frame.txt");
    // for(auto kf : keyframe_pose){
    //     keyframe_file << kf[0] << " "<< kf[1] << " "<< kf[2] << " "<< kf[3] << " "
    //                       << kf[4] << " "<< kf[5] << " "<< kf[6] << " "<< std::endl;
    // }
    // keyframe_file.close();
    pcl::PointCloud<pcl::PointXYZI> surf_points, corner_points;
    surf_points = *laserCloudSurfFromMap;
    corner_points = *laserCloudCornerFromMap;
    fout_out.close();
    fout_pre.close();
    if (surf_points.size() > 0 && corner_points.size() > 0) 
    {
    pcl::PCDWriter pcd_writer;
    std::cout << "saving...";
    pcd_writer.writeBinary(surf_filename, surf_points);
    pcd_writer.writeBinary(corner_filename, corner_points);
    pcd_writer.writeBinary(all_points_filename, *laserCloudFullResColor_pcd);
    }
    else
    {
        if (!T1.empty())
        {
            plt::named_plot("time consumed",T1,s_plot);
            // plt::named_plot("R_residual",T1,s_plot2);
            // plt::named_plot("T_residual",T1,s_plot3);
            plt::legend();
            plt::show();
            plt::pause(0.5);
            plt::close();
            // plt::save("/home/xw/catkin_like_loam/src/LIEK_LOAM/a.png");
        }
        std::cout << "no points saved";
    }
    //--------------------------
    //  loss_output.close();
  return 0;
}
