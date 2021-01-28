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
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <common_lib.h>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fast_lio/States.h>
#include <geometry_msgs/Vector3.h>
#include <FOV_Checker/FOV_Checker.h>

#ifdef USE_ikdtree
#include <kd_tree/kd_tree.h>
#else
#include <pcl/kdtree/kdtree_flann.h>
#endif

#ifndef DEPLOY
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

#define INIT_TIME           (0.5)
#define LASER_POINT_COV     (0.0015)
#define NUM_MATCH_POINTS    (5)

#define MAXN 360000

std::string root_dir = ROOT_DIR;

int iterCount = 0;
int NUM_MAX_ITERATIONS  = 0;
int FOV_RANGE = 3;  // range of FOV = FOV_RANGE * cube_len
int laserCloudCenWidth  = 24;
int laserCloudCenHeight = 24;
int laserCloudCenDepth  = 24;

int laserCloudValidNum    = 0;
int effct_feat_num      = 0;

const int laserCloudWidth  = 48;
const int laserCloudHeight = 48;
const int laserCloudDepth  = 48;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

// std::vector<double> T1, T2, s_plot, s_plot2, s_plot3, s_plot4, s_plot5, s_plot6;
double T1[MAXN], T2[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN];
int time_log_counter = 0;
/// IMU relative variables
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
bool lidar_pushed = false;
bool flg_exit = false;
bool flg_reset = false;
bool dense_map_en = true;

/// Buffers for measurements
double cube_len = 0.0;
double lidar_end_time = 0.0;
double last_timestamp_lidar = -1;
double last_timestamp_imu   = -1;
double HALF_FOV_COS = 0.0;
double FOV_DEG = 0.0;
double res_mean_last = 0.05;
double total_distance = 0.0;
auto   position_last  = Zero3d;
double copy_time, readd_time, fov_check_time, readd_box_time, delete_box_time;
double kdtree_incremental_time, kdtree_search_time;

std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr cube_points_add(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
std::vector<BoxPointType> cub_needrm;
std::vector<BoxPointType> cub_needad;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

//all points
PointCloudXYZI::Ptr featsArray[laserCloudNum];
bool                _last_inFOV[laserCloudNum];
bool                now_inFOV[laserCloudNum];
bool                cube_updated[laserCloudNum];
int laserCloudValidInd[laserCloudNum];
int feats_down_size = 0;

#ifdef USE_ikdtree
KD_TREE ikdtree;
#else
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
#endif
#ifdef USE_FOV_Checker
FOV_Checker fov_checker;
double FOV_depth;
double theta;
Eigen::Vector3d FOV_axis;
Eigen::Vector3d FOV_pos;
vector<BoxPointType> boxes;
#endif

Eigen::Vector3f XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
Eigen::Vector3f XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);

//estimator inputs and output;
MeasureGroup Measures;
StatesGroup  state;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

void SigHandle(int sig)
{
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

//project the lidar scan to world frame
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
    Eigen::Vector3d p_body(pi[0], pi[1], pi[2]);
    Eigen::Vector3d p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity*10000;
}

int cube_ind(const int &i, const int &j, const int &k)
{
    return (i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
}

bool CenterinFOV(Eigen::Vector3f cube_p)
{                                
    Eigen::Vector3f dis_vec = state.pos_end.cast<float>() - cube_p;
    float squaredSide1 = dis_vec.transpose() * dis_vec;

    if(squaredSide1 < 0.4 * cube_len * cube_len) return true;

    dis_vec = XAxisPoint_world.cast<float>() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;

    float ang_cos = fabs(squaredSide1 <= 3) ? 1.0 :
        (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));
    
    return ((ang_cos > HALF_FOV_COS)? true : false);
}

bool CornerinFOV(Eigen::Vector3f cube_p)
{                                
    Eigen::Vector3f dis_vec = state.pos_end.cast<float>() - cube_p;
    float squaredSide1 = dis_vec.transpose() * dis_vec;

    dis_vec = XAxisPoint_world.cast<float>() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;

    float ang_cos = fabs(squaredSide1 <= 3) ? 1.0 :
        (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));
    
    return ((ang_cos > HALF_FOV_COS)? true : false);
}

void lasermap_fov_segment()
{
    laserCloudValidNum = 0;

    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);

    int centerCubeI = int((state.pos_end(0) + 0.5 * cube_len) / cube_len) + laserCloudCenWidth;
    int centerCubeJ = int((state.pos_end(1) + 0.5 * cube_len) / cube_len) + laserCloudCenHeight;
    int centerCubeK = int((state.pos_end(2) + 0.5 * cube_len) / cube_len) + laserCloudCenDepth;

    if (state.pos_end(0) + 0.5 * cube_len < 0) centerCubeI--;
    if (state.pos_end(1) + 0.5 * cube_len < 0) centerCubeJ--;
    if (state.pos_end(2) + 0.5 * cube_len < 0) centerCubeK--;

    bool last_inFOV_flag = 0;
    int  cube_index = 0;
    std::vector<BoxPointType> ().swap(cub_needrm);
    std::vector<BoxPointType> ().swap(cub_needad);

    T2[time_log_counter] = Measures.lidar_beg_time;
    double t_begin = omp_get_wtime();

    std::cout<<"centerCubeIJK: "<<centerCubeI<<" "<<centerCubeJ<<" "<<centerCubeK<<std::endl;

    while (centerCubeI < FOV_RANGE + 1)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = laserCloudWidth - 1;

                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[cube_ind(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--) {
                    featsArray[cube_ind(i, j, k)]  = featsArray[cube_ind(i-1, j, k)];
                    _last_inFOV[cube_ind(i, j, k)] = _last_inFOV[cube_ind(i-1, j, k)];
                }

                featsArray[cube_ind(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[cube_ind(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeI++;
        laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - (FOV_RANGE + 1)) {
        for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
                int i = 0;

                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[cube_ind(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--) {
                    featsArray[cube_ind(i, j, k)]  = featsArray[cube_ind(i+1, j, k)];
                    _last_inFOV[cube_ind(i, j, k)] = _last_inFOV[cube_ind(i+1, j, k)];
                }

                featsArray[cube_ind(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[cube_ind(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        laserCloudCenWidth--;
    }

    while (centerCubeJ < (FOV_RANGE + 1)) {
        for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
                int j = laserCloudHeight - 1;

                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[cube_ind(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--) {
                    featsArray[cube_ind(i, j, k)]  = featsArray[cube_ind(i, j-1, k)];
                    _last_inFOV[cube_ind(i, j, k)] = _last_inFOV[cube_ind(i, j-1, k)];
                }

                featsArray[cube_ind(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[cube_ind(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - (FOV_RANGE + 1)) {
        for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
                int j = 0;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[cube_ind(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--) {
                    featsArray[cube_ind(i, j, k)]  = featsArray[cube_ind(i, j+1, k)];
                    _last_inFOV[cube_ind(i, j, k)] = _last_inFOV[cube_ind(i, j+1, k)];
                }

                featsArray[cube_ind(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[cube_ind(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
    }

    while (centerCubeK < (FOV_RANGE + 1)) {
        for (int i = 0; i < laserCloudWidth; i++) {
            for (int j = 0; j < laserCloudHeight; j++) {
                int k = laserCloudDepth - 1;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[cube_ind(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--) {
                    featsArray[cube_ind(i, j, k)]  = featsArray[cube_ind(i, j, k-1)];
                    _last_inFOV[cube_ind(i, j, k)] = _last_inFOV[cube_ind(i, j, k-1)];
                }

                featsArray[cube_ind(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[cube_ind(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        laserCloudCenDepth++;
    }

    while (centerCubeK >= laserCloudDepth - (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = 0;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[cube_ind(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--) {
                    featsArray[cube_ind(i, j, k)]  = featsArray[cube_ind(i, j, k+1)];
                    _last_inFOV[cube_ind(i, j, k)] = _last_inFOV[cube_ind(i, j, k+1)];
                }

                featsArray[cube_ind(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[cube_ind(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeK--;
        laserCloudCenDepth--;
    }

    cube_points_add->clear();
    featsFromMap->clear();
    memset(now_inFOV, 0, sizeof(now_inFOV));  
    copy_time = omp_get_wtime() - t_begin;      
    double fov_check_begin = omp_get_wtime();
    #ifdef USE_FOV_Checker
        BoxPointType env_box;
        env_box.vertex_min[0] = max(centerCubeI - FOV_RANGE, 0) * cube_len - laserCloudWidth * cube_len / 2.0;
        env_box.vertex_max[0] = min(centerCubeI + FOV_RANGE,laserCloudWidth) * cube_len - laserCloudWidth * cube_len / 2.0;
        env_box.vertex_min[1] = max(centerCubeJ - FOV_RANGE,0) * cube_len - laserCloudHeight * cube_len / 2.0;
        env_box.vertex_max[1] = min(centerCubeJ + FOV_RANGE,laserCloudHeight) * cube_len - laserCloudHeight * cube_len / 2.0;
        env_box.vertex_min[2] = max(centerCubeK - FOV_RANGE, 0) * cube_len - laserCloudDepth * cube_len / 2.0;
        env_box.vertex_max[2] = min(centerCubeK + FOV_RANGE, laserCloudDepth) * cube_len - laserCloudDepth * cube_len /2.0;
        fov_checker.Set_Env(env_box);
        fov_checker.Set_BoxLength(cube_len);
        FOV_depth = FOV_RANGE * cube_len;
        theta = ceil(FOV_DEG/2.0)/180 * PI_M;
        Eigen::Vector3d tmp = state.rot_end.transpose() * Eigen::Vector3d(1,0,0);
        FOV_axis(0) = tmp(0);
        FOV_axis(1) = -tmp(1);
        FOV_axis(2) = -tmp(2);
        FOV_pos = state.pos_end;        
        fov_checker.check_fov(FOV_pos, FOV_axis, theta, FOV_depth, boxes);
        // FILE *fp;
        // fp = fopen("/home/ecstasy/catkin_ws/fov_data.csv","a");
        // fprintf(fp,"%d,",int(boxes.size()));    
        // fprintf(fp,"%f,%f,%f,",tmp(0), tmp(1), tmp(2));
        int cube_i, cube_j, cube_k;
        for (int i = 0; i < boxes.size(); i++){
            cube_i = floor((boxes[i].vertex_min[0] + eps_value + laserCloudWidth * cube_len / 2.0) / cube_len);
            cube_j = floor((boxes[i].vertex_min[1] + eps_value + laserCloudHeight * cube_len / 2.0)/ cube_len);
            cube_k = floor((boxes[i].vertex_min[2] + eps_value + laserCloudDepth * cube_len / 2.0) / cube_len);
            cube_index = cube_ind(cube_i, cube_j, cube_k);
            #ifdef USE_ikdtree
                *cube_points_add += *featsArray[cube_index];
                featsArray[cube_index]->clear();            
                now_inFOV[cube_index] = true;
                if (!_last_inFOV[cube_index]) {
                    cub_needad.push_back(boxes[i]);
                    laserCloudValidInd[laserCloudValidNum] = cube_index;
                    laserCloudValidNum++;
                    _last_inFOV[cube_index] = true;
                }
            #else
                *featsFromMap += *featsArray[cube_index];
                laserCloudValidInd[laserCloudValidNum] = cube_index;
                laserCloudValidNum++;
            #endif
        }
        #ifdef USE_ikdtree
            BoxPointType rm_box;
            for (int i = 0; i < laserCloudNum; i++){
                if (_last_inFOV[i] && !now_inFOV[i]){
                    cube_i = i % laserCloudWidth;
                    cube_j = (i % (laserCloudWidth * laserCloudHeight))/laserCloudWidth;
                    cube_k = i / (laserCloudWidth * laserCloudHeight);
                    rm_box.vertex_min[0] = cube_i * cube_len - laserCloudWidth * cube_len / 2.0;
                    rm_box.vertex_max[0] = rm_box.vertex_min[0] + cube_len;
                    rm_box.vertex_min[1] = cube_j * cube_len - laserCloudHeight * cube_len / 2.0;
                    rm_box.vertex_max[1] = rm_box.vertex_min[1] + cube_len;
                    rm_box.vertex_min[2] = cube_k * cube_len - laserCloudDepth * cube_len / 2.0;
                    rm_box.vertex_max[2] = rm_box.vertex_min[2] + cube_len;
                    cub_needrm.push_back(rm_box);
                    _last_inFOV[i] = false;
                }
            }
        #endif
        // fprintf(fp,"\n");
        // fclose(fp);
    #else
        for (int i = centerCubeI - FOV_RANGE; i <= centerCubeI + FOV_RANGE; i++) 
        {
            for (int j = centerCubeJ - FOV_RANGE; j <= centerCubeJ + FOV_RANGE; j++) 
            {
                for (int k = centerCubeK - FOV_RANGE; k <= centerCubeK + FOV_RANGE; k++) 
                {
                    if (i >= 0 && i < laserCloudWidth &&
                            j >= 0 && j < laserCloudHeight &&
                            k >= 0 && k < laserCloudDepth) 
                    {
                        Eigen::Vector3f center_p(cube_len * (i - laserCloudCenWidth), \
                                                cube_len * (j - laserCloudCenHeight), \
                                                cube_len * (k - laserCloudCenDepth));

                        float check1, check2;
                        float squaredSide1, squaredSide2;
                        float ang_cos = 1;
                        bool &last_inFOV = _last_inFOV[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        bool inFOV = CenterinFOV(center_p);

                        for (int ii = -1; (ii <= 1) && (!inFOV); ii += 2) 
                        {
                            for (int jj = -1; (jj <= 1) && (!inFOV); jj += 2) 
                            {
                                for (int kk = -1; (kk <= 1) && (!inFOV); kk += 2) 
                                {
                                    Eigen::Vector3f corner_p(cube_len * ii, cube_len * jj, cube_len * kk);
                                    corner_p = center_p + 0.5 * corner_p;
                                    
                                    inFOV = CornerinFOV(corner_p);
                                }
                            }
                        }

                        now_inFOV[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = inFOV;

                    #ifdef USE_ikdtree
                        /*** readd cubes and points ***/
                        if (inFOV)
                        {
                            int center_index = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                            *cube_points_add += *featsArray[center_index];
                            featsArray[center_index]->clear();
                            if (!last_inFOV)
                            {
                                BoxPointType cub_points;
                                for(int i = 0; i < 3; i++)
                                {
                                    cub_points.vertex_max[i] = center_p[i] + 0.5 * cube_len;
                                    cub_points.vertex_min[i] = center_p[i] - 0.5 * cube_len;
                                }
                                cub_needad.push_back(cub_points);
                                laserCloudValidInd[laserCloudValidNum] = center_index;
                                laserCloudValidNum ++;
                                // std::cout<<"readd center: "<<center_p.transpose()<<std::endl;
                            }
                        }

                    #else
                        if (inFOV)
                        {
                            int center_index = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                            *featsFromMap += *featsArray[center_index];
                            laserCloudValidInd[laserCloudValidNum] = center_index;
                            laserCloudValidNum++;
                        }
                        last_inFOV = inFOV;
                    #endif
                    }
                }
            }
        }

        #ifdef USE_ikdtree
        /*** delete cubes ***/
        for (int i = 0; i < laserCloudWidth; i++) 
        {
            for (int j = 0; j < laserCloudHeight; j++) 
            {
                for (int k = 0; k < laserCloudDepth; k++) 
                {
                    int ind = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                    if((!now_inFOV[ind]) && _last_inFOV[ind])
                    {
                        BoxPointType cub_points;
                        Eigen::Vector3f center_p(cube_len * (i - laserCloudCenWidth),\
                                                cube_len * (j - laserCloudCenHeight),\
                                                cube_len * (k - laserCloudCenDepth));
                        // std::cout<<"center_p: "<<center_p.transpose()<<std::endl;

                        for(int i = 0; i < 3; i++)
                        {
                            cub_points.vertex_max[i] = center_p[i] + 0.5 * cube_len;
                            cub_points.vertex_min[i] = center_p[i] - 0.5 * cube_len;
                        }
                        cub_needrm.push_back(cub_points);
                    }
                    _last_inFOV[ind] = now_inFOV[ind];
                }
            }
        }
        #endif
    #endif
    fov_check_time = omp_get_wtime()- fov_check_begin;    

    double readd_begin = omp_get_wtime();
#ifdef USE_ikdtree
    if(cub_needrm.size() > 0)               ikdtree.Delete_Point_Boxes(cub_needrm);
    delete_box_time = omp_get_wtime() - readd_begin;
    // s_plot4.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if(cub_needad.size() > 0)               ikdtree.Add_Point_Boxes(cub_needad); 
    readd_box_time = omp_get_wtime() - readd_begin - delete_box_time;
    // s_plot5.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if(cube_points_add->points.size() > 0)  ikdtree.Add_Points(cube_points_add->points, true);
    readd_time = omp_get_wtime()- readd_begin - delete_box_time - readd_box_time;
    int ikdtree_size = ikdtree.size();
    s_plot4[time_log_counter] = float(ikdtree_size);
    if (ikdtree_size>0) {
        s_plot6[time_log_counter] = float(cube_points_add->points.size())/float(ikdtree_size);
    } else {
        s_plot6[time_log_counter] = 0.0;
    }
#endif
}

void feat_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    // std::cout<<"got feature"<<std::endl;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    lidar_buffer.push_back(msg);
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    // std::cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<std::endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar.reset(new PointCloudXYZI());
        pcl::fromROSMsg(*(lidar_buffer.front()), *(meas.lidar));
        if(meas.lidar->points.size() <= 1)
        {
            lidar_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();
        std::cout<<"scan size: "<<meas.lidar->points.size()<<std::endl;
        std::cout<<"max offset time: "<<meas.lidar->points.back().curvature<<std::endl;
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time + 0.02) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    lidar_pushed = false;
    // if (meas.imu.empty()) return false;
    // std::cout<<"[IMU Sycned]: "<<imu_time<<" "<<lidar_end_time<<std::endl;
    return true;
}

void map_incremental()
{
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    }
#ifdef USE_ikdtree
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    
    for (int i = 0; i < points_history.size(); i++)
    {
        PointType &pointSel = points_history[i];

        int cubeI = int((pointSel.x + 0.5 * cube_len) / cube_len) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 0.5 * cube_len) / cube_len) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + 0.5 * cube_len) / cube_len) + laserCloudCenDepth;

        if (pointSel.x + 0.5 * cube_len < 0) cubeI--;
        if (pointSel.y + 0.5 * cube_len < 0) cubeJ--;
        if (pointSel.z + 0.5 * cube_len < 0) cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth) 
        {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            featsArray[cubeInd]->push_back(pointSel);
            
        }
    }

    ikdtree.Add_Points(feats_down_world->points, true);
#else
    bool cube_updated[laserCloudNum] = {0};
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &pointSel = feats_down_world->points[i];

        int cubeI = int((pointSel.x + 0.5 * cube_len) / cube_len) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 0.5 * cube_len) / cube_len) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + 0.5 * cube_len) / cube_len) + laserCloudCenDepth;

        if (pointSel.x + 0.5 * cube_len < 0) cubeI--;
        if (pointSel.y + 0.5 * cube_len < 0) cubeJ--;
        if (pointSel.z + 0.5 * cube_len < 0) cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth) {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            featsArray[cubeInd]->push_back(pointSel);
            cube_updated[cubeInd] = true;
        }
    }
    for (int i = 0; i < laserCloudValidNum; i++)
    {
        int ind = laserCloudValidInd[i];

        if(cube_updated[ind])
        {
            downSizeFilterMap.setInputCloud(featsArray[ind]);
            downSizeFilterMap.filter(*featsArray[ind]);
        }
    }
#endif
}

void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)
{
    PointCloudXYZI::Ptr laserCloudFullRes;
    laserCloudFullRes = dense_map_en ? feats_undistort : feats_down_body;

    int size = laserCloudFullRes->points.size();

    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
    laserCloudmsg.header.frame_id = "/camera_init";
    pubLaserCloudFullRes.publish(laserCloudmsg);
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
    laserCloudFullRes3.header.frame_id = "/camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time::now();
    laserCloudMap.header.frame_id = "/camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.position.x = state.pos_end(0);
    out.position.y = state.pos_end(1);
    out.position.z = state.pos_end(2);
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "/camera_init";
    odomAftMapped.child_frame_id = "/aft_mapped";
    odomAftMapped.header.stamp = ros::Time::now();//ros::Time().fromSec(last_timestamp_lidar);
    set_posestamp(odomAftMapped.pose.pose);
    // static tf::TransformBroadcaster br;
    // tf::Transform                   transform;
    // tf::Quaternion                  q;
    // transform.setOrigin(tf::Vector3(state.pos_end(0), state.pos_end(1), state.pos_end(2)));
    // q.setW(geoQuat.w);
    // q.setX(geoQuat.x);
    // q.setY(geoQuat.y);
    // q.setZ(geoQuat.z);
    // transform.setRotation( q );
    // br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped" ) );
    pubOdomAftMapped.publish(odomAftMapped);
}

void publish_mavros(const ros::Publisher & mavros_pose_publisher)
{
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "/camera_odom_frame";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "/camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    ros::Subscriber sub_pcl = nh.subscribe("/laser_cloud_flat", 20000, feat_points_cbk);
    ros::Subscriber sub_imu = nh.subscribe("/livox/imu", 20000, imu_cbk);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100);
    ros::Publisher pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/aft_mapped_to_init", 10);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 10);
#ifdef DEPLOY
    ros::Publisher mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
#endif
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="/camera_init";

    /*** variables definition ***/
    bool flg_EKF_inited = 0, flg_EKF_converged = 0;
    std::string map_file_path;
    int effect_feat_num = 0, frame_num = 0;
    double filter_size_corner_min, filter_size_surf_min, filter_size_map_min, fov_deg,\
           deltaT, deltaR, aver_time_consu = 0, first_lidar_time = 0;
    Eigen::Vector3d rot_add, t_add, euler_cur;
    Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
    Eigen::Matrix<double,DIM_OF_STATES,DIM_OF_STATES> G, H_T_H, I_STATE;
    StatesGroup state_propagat;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    PointType pointOri, pointSel, coeff;
    PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

    /*** variables initialize ***/
    ros::param::get("~dense_map_enable",dense_map_en);
    ros::param::get("~max_iteration",NUM_MAX_ITERATIONS);
    ros::param::get("~map_file_path",map_file_path);
    ros::param::get("~fov_degree",fov_deg);
    ros::param::get("~filter_size_corner",filter_size_corner_min);
    ros::param::get("~filter_size_surf",filter_size_surf_min);
    ros::param::get("~filter_size_map",filter_size_map_min);
    ros::param::get("~cube_side_length",cube_len);
    
    FOV_DEG = fov_deg + 10.0;
    HALF_FOV_COS = std::cos((fov_deg + 10.0) * 0.5 * PI_M / 180.0);

    for (int i = 0; i < laserCloudNum; i++)
    {
        featsArray[i].reset(new PointCloudXYZI());
    }

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    #ifdef USE_FOV_Checker
        BoxPointType env_box;
        env_box.vertex_min[0] = -laserCloudWidth/2.0 * cube_len;
        env_box.vertex_max[0] = laserCloudWidth/2.0 * cube_len;
        env_box.vertex_min[1] = -laserCloudHeight/2.0 * cube_len;
        env_box.vertex_max[1] = laserCloudHeight/2.0 * cube_len;
        env_box.vertex_min[2] = -laserCloudDepth/2.0 * cube_len;
        env_box.vertex_max[2] = laserCloudDepth/2.0 * cube_len;
        fov_checker.Set_Env(env_box);
    #endif
    std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

    /*** debug record ***/
    std::ofstream fout_pre, fout_out;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),std::ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),std::ios::out);
    if (fout_pre && fout_out)
        std::cout << "~~~~"<<ROOT_DIR<<" file opened" << std::endl;
    else
        std::cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << std::endl;

//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(sync_packages(Measures)) 
        {
            if (flg_reset)
            {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, match_time, solve_start, solve_time, pca_time, svd_time;
            match_time = 0;
            kdtree_search_time = 0;
            solve_time = 0;
            pca_time   = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, state, feats_undistort);
            state_propagat = state;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                first_lidar_time = Measures.lidar_beg_time;
                std::cout<<"FAST-LIO not ready"<<std::endl;
                continue;
            }

        #ifdef DEBUG_PRINT
            euler_cur = RotMtoEuler(state.rot_end);
            fout_pre << std::setw(10) << Measures.lidar_beg_time << " " << euler_cur.transpose()*57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() \
            <<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<< std::endl;
            std::cout<<"current lidar time "<<Measures.lidar_beg_time<<" "<<"first lidar time "<<first_lidar_time<<std::endl;
            std::cout<<"pre-integrated states: "<<euler_cur.transpose()*57.3<<" "<<state.pos_end.transpose()<<" "<<state.vel_end.transpose()<<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<<std::endl;
        #endif
            
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();
            
            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);

        #ifdef USE_ikdtree
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_body->points.size() > 1)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    ikdtree.Build(feats_down_body->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.size();
        #else
            if(featsFromMap->points.empty())
            {
                downSizeFilterMap.setInputCloud(feats_down_body);
            }
            else
            {
                downSizeFilterMap.setInputCloud(featsFromMap);
            }
            downSizeFilterMap.filter(*featsFromMap);
            int featsFromMapNum = featsFromMap->points.size();
        #endif
            feats_down_size = feats_down_body->points.size();
            std::cout<<"[ mapping ]: Raw feature num: "<<feats_undistort->points.size()<<" downsamp num "<<feats_down_size<<" Map num: "<<featsFromMapNum<<" laserCloudValidNum "<<laserCloudValidNum<<std::endl;

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            std::vector<double> res_last(feats_down_size, 1000.0); // initial
            
            t1 = omp_get_wtime(); 
            if (featsFromMapNum <= 5) continue;

        #ifdef USE_ikdtree
            if(0)
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }
        #else
            kdtreeSurfFromMap->setInputCloud(featsFromMap);
        #endif

            std::vector<bool> point_selected_surf(feats_down_size, true);
            std::vector<std::vector<int>> pointSearchInd_surf(feats_down_size);
            std::vector<PointVector> Nearest_Points(feats_down_size);
            
            int  rematch_num = 0;
            bool nearest_search_en = true;

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) 
            {
                match_start = omp_get_wtime();
                laserCloudOri->clear();
                corr_normvect->clear();

                /** closest surface search and residual computation **/
                omp_set_num_threads(4);
                #pragma omp parallel for
                for (int i = 0; i < feats_down_size; i++)
                {
                    PointType &point_body  = feats_down_body->points[i];
                    PointType &point_world = feats_down_world->points[i];
                    double search_start = omp_get_wtime();
                    /* transform to world frame */
                    pointBodyToWorld(&point_body, &point_world);
                    std::vector<float> pointSearchSqDis_surf;
                #ifdef USE_ikdtree
                    auto &points_near = Nearest_Points[i];
                #else
                    auto &points_near = pointSearchInd_surf[i];
                #endif
                    
                    if (nearest_search_en)
                    {
                        point_selected_surf[i] = true;
                        /** Find the closest surfaces in the map **/
                    #ifdef USE_ikdtree
                        ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);
                    #else
                        kdtreeSurfFromMap->nearestKSearch(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);
                    #endif
                        float max_distance = pointSearchSqDis_surf[NUM_MATCH_POINTS - 1];
                    
                        if (max_distance > 3)
                        {
                            point_selected_surf[i] = false;
                        }
                    }
                    kdtree_search_time = omp_get_wtime() - search_start;
                    if (point_selected_surf[i] == false) continue;

                    // match_time += omp_get_wtime() - match_start;

                    double pca_start = omp_get_wtime();

                    /// PCA (using minimum square method)
                    cv::Mat matA0(NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all(0));
                    cv::Mat matB0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(-1));
                    cv::Mat matX0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(0));
                    
                    for (int j = 0; j < NUM_MATCH_POINTS; j++)
                    {
                    #ifdef USE_ikdtree
                        matA0.at<float>(j, 0) = points_near[j].x;
                        matA0.at<float>(j, 1) = points_near[j].y;
                        matA0.at<float>(j, 2) = points_near[j].z;
                    #else
                        matA0.at<float>(j, 0) = featsFromMap->points[points_near[j]].x;
                        matA0.at<float>(j, 1) = featsFromMap->points[points_near[j]].y;
                        matA0.at<float>(j, 2) = featsFromMap->points[points_near[j]].z;
                    #endif
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

                    //ps is the norm of the plane norm_vec vector
                    //pd is the distance from point to plane
                    float ps = sqrt(pa * pa + pb * pb + pc * pc);
                    pa /= ps;
                    pb /= ps;
                    pc /= ps;
                    pd /= ps;

                    bool planeValid = true;
                    for (int j = 0; j < NUM_MATCH_POINTS; j++)
                    {
                    #ifdef USE_ikdtree
                        if (fabs(pa * points_near[j].x +
                                    pb * points_near[j].y +
                                    pc * points_near[j].z + pd) > 0.1)
                    #else
                        if (fabs(pa * featsFromMap->points[points_near[j]].x +
                                    pb * featsFromMap->points[points_near[j]].y +
                                    pc * featsFromMap->points[points_near[j]].z + pd) > 0.1)
                    #endif
                        {
                            planeValid = false;
                            point_selected_surf[i] = false;
                            break;
                        }
                    }

                    if (planeValid) 
                    {
                        //loss fuction
                        float pd2 = pa * point_world.x + pb * point_world.y + pc * point_world.z + pd;
                        //if(fabs(pd2) > 0.1) continue;
                        float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(point_world.x * point_world.x + point_world.y * point_world.y + point_world.z * point_world.z));

                        if (s > 0.85)
                        {
                            point_selected_surf[i] = true;
                            normvec->points[i].x = pa;
                            normvec->points[i].y = pb;
                            normvec->points[i].z = pc;
                            normvec->points[i].intensity = pd2;
                            
                            res_last[i] = std::abs(pd2);
                        }
                        else
                        {
                            point_selected_surf[i] = false;
                        }
                    }

                    pca_time += omp_get_wtime() - pca_start;
                }

                double total_residual = 0.0;
                effct_feat_num = 0;

                for (int i = 0; i < feats_down_size; i++)
                {
                    if (point_selected_surf[i] && (res_last[i] <= 2.0))
                    {
                        laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                        corr_normvect->points[effct_feat_num] = normvec->points[i];
                        total_residual += res_last[i];
                        effct_feat_num ++;
                    }
                }

                res_mean_last = total_residual / effct_feat_num;
                std::cout << "[ mapping ]: Effective feature num: "<<effct_feat_num<<" res_mean_last "<<res_mean_last<<std::endl;

                match_time += omp_get_wtime() - match_start;
                solve_start = omp_get_wtime();
                
                /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                Eigen::MatrixXd Hsub(effct_feat_num, 6);
                Eigen::VectorXd meas_vec(effct_feat_num);
                Hsub.setZero();

                for (int i = 0; i < effct_feat_num; i++)
                {
                    const PointType &laser_p  = laserCloudOri->points[i];
                    Eigen::Vector3d point_this(laser_p.x, laser_p.y, laser_p.z);
                    point_this += Lidar_offset_to_IMU;
                    Eigen::Matrix3d point_crossmat;
                    point_crossmat<<SKEW_SYM_MATRX(point_this);

                    /*** get the normal vector of closest surface/corner ***/
                    const PointType &norm_p = corr_normvect->points[i];
                    Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

                    /*** calculate the Measuremnt Jacobian matrix H ***/
                    Eigen::Vector3d A(point_crossmat * state.rot_end.transpose() * norm_vec);
                    Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

                    /*** Measuremnt: distance to the closest surface/corner ***/
                    meas_vec(i) = - norm_p.intensity;
                }

                Eigen::MatrixXd K(DIM_OF_STATES, effct_feat_num);
                
                /*** Iterative Kalman Filter Update ***/
                flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
                if (!flg_EKF_inited)
                {
                    std::cout<<"||||||||||Initiallizing LiDar||||||||||"<<std::endl;
                    /*** only run in initialization period ***/
                    Eigen::MatrixXd H_init(Eigen::Matrix<double, 9, DIM_OF_STATES>::Zero());
                    Eigen::MatrixXd z_init(Eigen::Matrix<double, 9, 1>::Zero());
                    H_init.block<3,3>(0,0)  = Eigen::Matrix3d::Identity();
                    H_init.block<3,3>(3,3)  = Eigen::Matrix3d::Identity();
                    H_init.block<3,3>(6,15) = Eigen::Matrix3d::Identity();
                    z_init.block<3,1>(0,0)  = - Log(state.rot_end);
                    z_init.block<3,1>(0,0)  = - state.pos_end;

                    auto H_init_T = H_init.transpose();
                    auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T + \
                                    0.0001 * Eigen::Matrix<double,9,9>::Identity()).inverse();
                    solution      = K_init * z_init;

                    solution.block<9,1>(0,0).setZero();
                    state += solution;
                    state.cov = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K_init * H_init) * state.cov;
                }
                else
                {
                    auto &&Hsub_T = Hsub.transpose();
                    H_T_H.block<6,6>(0,0) = Hsub_T * Hsub;
                    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 = \
                                (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse();
                    K = K_1.block<DIM_OF_STATES,6>(0,0) * Hsub_T;

                    auto vec = state_propagat - state;
                    solution = K * meas_vec + vec - K * Hsub * vec.block<6,1>(0,0);
                    state += solution;

                    rot_add = solution.block<3,1>(0,0);
                    t_add   = solution.block<3,1>(3,0);

                    flg_EKF_converged = false;

                    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015))
                    {
                        flg_EKF_converged = true;
                    }

                    deltaR = rot_add.norm() * 57.3;
                    deltaT = t_add.norm() * 100;
                }

                euler_cur = RotMtoEuler(state.rot_end);

                #ifdef DEBUG_PRINT
                std::cout<<"update: R"<<euler_cur.transpose()*57.3<<" p "<<state.pos_end.transpose()<<" v "<<state.vel_end.transpose()<<" bg"<<state.bias_g.transpose()<<" ba"<<state.bias_a.transpose()<<std::endl;
                std::cout<<"dR & dT: "<<deltaR<<" "<<deltaT<<" res norm:"<<res_mean_last<<std::endl;
                #endif        

                /*** Rematch Judgement ***/
                nearest_search_en = false;
                if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2))))
                {
                    nearest_search_en = true;
                    rematch_num ++;
                }

                /*** Convergence Judgements and Covariance Update ***/
                if (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))
                {
                    if (flg_EKF_inited)
                    {
                        /*** Covariance Update ***/
                        G.block<DIM_OF_STATES,6>(0,0) = K * Hsub;
                        state.cov = (I_STATE - G) * state.cov;
                        total_distance += (state.pos_end - position_last).norm();
                        position_last = state.pos_end;
                        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                    (euler_cur(0), euler_cur(1), euler_cur(2));
                        
                        std::cout<<"position: "<<state.pos_end.transpose()<<" total distance: "<<total_distance<<std::endl;
                    }
                    solve_time += omp_get_wtime() - solve_start;
                    break;
                }
                solve_time += omp_get_wtime() - solve_start;
            }
            std::cout<<"[ mapping ]: iteration count: "<<iterCount+1<<std::endl;

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            kdtree_incremental_time = t5 - t3 + readd_time + readd_box_time + delete_box_time;
            /******* Publish functions:  *******/
            publish_frame_world(pubLaserCloudFullRes);

            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);
            publish_odometry(pubOdomAftMapped);
            publish_path(pubPath);
            #ifdef DEPLOY
            publish_mavros(mavros_pose_publisher);
            #endif

            /*** Debug variables ***/
            frame_num ++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            // aver_time_consu = aver_time_consu * 0.8 + (t5 - t0) * 0.2;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = aver_time_consu;
            s_plot2[time_log_counter] = kdtree_incremental_time;
            s_plot3[time_log_counter] = kdtree_search_time;
            s_plot5[time_log_counter] = t5 - t0;
            time_log_counter ++;
            std::cout<<"[ mapping ]: time: fov_check "<< fov_check_time <<" fov_check and readd: "<<t1-t0<<" match "<<match_time<<" solve "<<solve_time<<" ICP "<<t3-t1<<" map incre "<<t5-t3<<" total "<<aver_time_consu<<std::endl;
            // fout_out << std::setw(10) << Measures.lidar_beg_time << " " << euler_cur.transpose()*57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() \
            // <<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<< std::endl;
            fout_out<<std::setw(8)<<effct_feat_num<<" "<<Measures.lidar_beg_time<<" "<<t2-t0<<" "<<match_time<<" "<<t5-t3<<" "<<t5-t0<<std::endl;
        }
        status = ros::ok();
        rate.sleep();
    }
    //--------------------------save map---------------
    // std::string surf_filename(map_file_path + "/surf.pcd");
    // std::string corner_filename(map_file_path + "/corner.pcd");
    // std::string all_points_filename(map_file_path + "/all_points.pcd");

    // PointCloudXYZI surf_points, corner_points;
    // surf_points = *featsFromMap;
    // fout_out.close();
    // fout_pre.close();
    // if (surf_points.size() > 0 && corner_points.size() > 0) 
    // {
    // pcl::PCDWriter pcd_writer;
    // std::cout << "saving...";
    // pcd_writer.writeBinary(surf_filename, surf_points);
    // pcd_writer.writeBinary(corner_filename, corner_points);
    // }

    #ifndef DEPLOY
    std::vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6;    
    FILE *fp;
    std::string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
    fp = fopen(log_dir.c_str(),"w");
    fprintf(fp,"time_stamp, average time, incremental time, search time,fov check time, total time, percentage of readd\n");
    for (int i = 0;i<time_log_counter; i++){
        fprintf(fp,"%f,%f,%f,%f,%f,%f,%f\n",T1[i],s_plot[i],s_plot2[i],s_plot3[i],s_plot4[i],s_plot5[i],s_plot6[i]);
        t.push_back(T1[i]);
        s_vec.push_back(s_plot[i]);
        s_vec2.push_back(s_plot2[i]);
        s_vec3.push_back(s_plot3[i]);
        s_vec4.push_back(s_plot4[i]);
        s_vec5.push_back(s_plot5[i]);
        s_vec6.push_back(s_plot6[i]);                        
    }
    fclose(fp);
    if (!t.empty())
    {
        plt::named_plot("incremental time",t,s_vec2);
        plt::named_plot("search_time",t,s_vec3);
        plt::named_plot("total time",t,s_vec5);
        plt::named_plot("average time",t,s_vec);
        plt::legend();
        plt::show();
        plt::pause(0.5);
        plt::close();
    }
    std::cout << "no points saved";
    #endif

    return 0;
}
