#include "FOV_Checker/FOV_Checker.h"

#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>

int main(int argc, char** argv){
    int cube_i, cube_j, cube_k, cube_index;
    Eigen::Vector3d FOV_axis(-0.999719,0.020994,0.010997);
    Eigen::Vector3d FOV_pos(0.221700,-0.103600,1.464300);
    const double theta = 0.750492;
    const double cube_len = 10;
    const double FOV_RANGE = 3;
    double FOV_depth = FOV_RANGE * cube_len;
    FOV_Checker fov_checker;
    BoxPointType env_box;
    env_box.vertex_min[0] = -60;
    env_box.vertex_min[1] = -60;
    env_box.vertex_min[2] = -60;
    env_box.vertex_max[0] = 60;
    env_box.vertex_max[1] = 60;
    env_box.vertex_max[2] = 60;
    fov_checker.Set_Env(env_box);
    fov_checker.Set_BoxLength(cube_len);
    vector<BoxPointType> boxes;
    BoxPointType box;
    box.vertex_min[0] = 0;
    box.vertex_min[1] = -20;
    box.vertex_min[2] = -20;
    box.vertex_max[0] = -20;
    box.vertex_max[1] = 60;
    box.vertex_max[2] = 0;
    Eigen::Vector3d line_p(20,-20,0);
    Eigen::Vector3d line_vec(0,80,0);
    double t1 = 0.0085;
    // bool s1 = fov_checker.check_line(FOV_pos, FOV_axis, theta, FOV_depth, line_p, line_vec);
    // printf("Check result is: %d \n", s1);
    fov_checker.check_fov(FOV_pos, FOV_axis, theta, FOV_depth, boxes);
    for (int i = 0; i< boxes.size(); i++){
        cube_i = floor((boxes[i].vertex_min[0] + eps_value + 48 * cube_len / 2.0) / cube_len);
        cube_j = floor((boxes[i].vertex_min[1] + eps_value + 48 * cube_len / 2.0)/ cube_len);
        cube_k = floor((boxes[i].vertex_min[2] + eps_value + 48 * cube_len / 2.0) / cube_len);
        cube_index = cube_i + cube_j * 48 + cube_k * 48 * 48;
        printf("(%d,%d,%d), %d ----",cube_i,cube_j,cube_k,cube_index);  
        printf("(%d,%d,%d)\n",cube_index % 48, int((cube_index % (48*48))/48),int(cube_index / (48*48)));
    }
    return 0;
}