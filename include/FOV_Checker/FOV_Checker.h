// Include Files
#pragma once
#include <math.h>
#include <cmath>
#include "kd_tree/kd_tree.h"
#include <Eigen/Core>
#include <algorithm>

#define eps_value 1e-6

struct PlaneType{
    Eigen::Vector3d p[4];
};

class FOV_Checker{
public:
    FOV_Checker();
    ~FOV_Checker();
    void Set_Env(BoxPointType env_param);
    void check_fov(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, double box_length, vector<BoxPointType> &boxes);
private:
    BoxPointType env;
    bool check_box(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, const BoxPointType box);
    bool check_line(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d line_p, Eigen::Vector3d line_vec);
    bool check_surface(Eigen::Vector3d cur_pose, Eigen::Vector3d axis,  double theta, double depth, PlaneType plane);
    bool check_point(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d point);
    bool check_box_in_env(BoxPointType box);
};

