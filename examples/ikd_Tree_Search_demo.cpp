/*
    Description: An example to introduce box search and radius search using ikd-Tree
    Author: Hyungtae Lim, Yixi Cai
*/
#include "ikd_Tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>
#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;
template class KD_TREE<pcl::PointXYZ>;

void colorize( const PointVector &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> &color) {
    int N = pc.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;

    for (int i = 0; i < N; ++i) {
        const auto &pt = pc[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

void generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths) {
    float &x_dist = box_lengths[0];
    float &y_dist = box_lengths[1];
    float &z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;
}

float test_dist(PointType a, PointType b)
{
    float dist = 0.0f;
    dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    return dist;
}

int main(int argc, char **argv) {
    /*** 1. Initialize k-d tree */
    KD_TREE<PointType>::Ptr kdtree_ptr(new KD_TREE<PointType>(0.3, 0.6, 0.2));
    KD_TREE<PointType>      &ikd_Tree        = *kdtree_ptr;

    /*** 2. Load point cloud data */
    pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud<PointType>);
    string filename = "../materials/hku_demo_pointcloud.pcd";
    if (pcl::io::loadPCDFile<PointType>(filename, *src) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    printf("Original: %d points are loaded\n", static_cast<int>(src->points.size()));

    /*** 3. Build ikd-Tree */
    auto start = chrono::high_resolution_clock::now();
    ikd_Tree.Build((*src).points);
    auto end      = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
    printf("Building tree takes: %0.3f ms\n", float(duration) / 1e3);
    printf("# of valid points: %d \n", ikd_Tree.validnum());

    /*** 4. Set a box region and search using box search */
    PointType center_pt;
    center_pt.x = 10.0;
    center_pt.y = 0.0;
    center_pt.z = 0.0;
    BoxPointType boxpoint;
    generate_box(boxpoint, center_pt, {5.00, 5.00, 50.0});

    start = chrono::high_resolution_clock::now();
    PointVector Searched_Points;
    ikd_Tree.Box_Search(boxpoint, Searched_Points);
    end  = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
    printf("Search Points by box takes: %0.3f ms with %d points\n", float(duration) / 1e3, static_cast<int>(Searched_Points.size()));

    /*** 5. Set a ball region and search using radius search */
    PointType ball_center_pt;
    ball_center_pt.x = 10.0;
    ball_center_pt.y = -5.0;
    ball_center_pt.z = 5.0;
    float radius = 7.5;
    start = chrono::high_resolution_clock::now();
    PointVector Searched_Points_radius;
    ikd_Tree.Radius_Search(ball_center_pt, radius, Searched_Points_radius);
    end = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
    printf("Search Points by radius takes: %0.3f ms with %d points\n", float(duration) / 1e3, int(Searched_Points_radius.size()));

    /*** Below codes are just for visualization */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr searched_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr searched_radius_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::visualization::PointCloudColorHandlerGenericField<PointType> src_color(src, "x");
    colorize(Searched_Points, *searched_colored, {255, 0, 0});
    colorize(Searched_Points_radius, *searched_radius_colored, {255, 0, 0});

    pcl::visualization::PCLVisualizer viewer0("Box Search");
    viewer0.addPointCloud<PointType>(src,src_color, "src");
    viewer0.addPointCloud<pcl::PointXYZRGB>(searched_colored, "searched");
    viewer0.setCameraPosition(-5, 30, 175,  0, 0, 0, 0.2, -1.0, 0.2);
    viewer0.setSize(1600, 900);

    pcl::visualization::PCLVisualizer viewer1("Radius Search");
    viewer1.addPointCloud<PointType>(src,src_color, "src");
    viewer1.addPointCloud<pcl::PointXYZRGB>(searched_radius_colored, "radius");
    viewer1.setCameraPosition(-5, 30, 175,  0, 0, 0, 0.2, -1.0, 0.2);
    viewer1.setSize(1600, 900);
             
    while (!viewer0.wasStopped() && !viewer1.wasStopped()){
        viewer0.spinOnce();
        viewer1.spinOnce();
    }

    return 0;
}