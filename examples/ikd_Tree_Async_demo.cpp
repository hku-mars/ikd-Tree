/*
    Description: An example to explain the asynchronous deletion on ikd-Tree
    Author: Hyungtae Lim
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

    /*** 4. Set a box region and delete the corresponding region */
    PointType center_pt;
    center_pt.x = 5.0;
    center_pt.y = -5.0;
    center_pt.z = 10.0;
    BoxPointType boxpoint;
    generate_box(boxpoint, center_pt, {10.0, 10.0, 20.0});

    start = chrono::high_resolution_clock::now();
    vector<BoxPointType> boxes = {boxpoint};
    int num_deleted = ikd_Tree.Delete_Point_Boxes(boxes);
    end  = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
    printf("Removal by box takes: %0.3f ms\n", float(duration) / 1e3);

    /*** NOTE. Check the removed points
     * In ikd-tree, the delete operation and the remove operation are not performed at the same time!!
     * Please refer the issue 14:
     * https://github.com/hku-mars/ikd-Tree/issues/14
     * It usually occurs when the target region is large!!!!
     * (i.e. the # of removed point are quite large)
     * */
    PointVector Removed_Points;
    ikd_Tree.acquire_removed_points(Removed_Points);
    printf("# of deleted points: %d\n", num_deleted);
    printf("# of removed points: %d\n", static_cast<int>(Removed_Points.size()));

    /*** 5. Check remaining points in ikd-Tree */
    pcl::PointCloud<PointType>::Ptr Remaining_Points(new pcl::PointCloud<PointType>);
    ikd_Tree.flatten(ikd_Tree.Root_Node, ikd_Tree.PCL_Storage, NOT_RECORD);
    Remaining_Points->points = ikd_Tree.PCL_Storage;
    printf("Finally, %d Points remain\n", static_cast<int>(Remaining_Points->points.size()));

    /*** Below codes are just for visualization */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::visualization::PointCloudColorHandlerGenericField<PointType> src_color(src, "x");
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> remaining_color(Remaining_Points, "x");

    colorize(Removed_Points, *removed_colored, {255, 0, 0});

    pcl::visualization::PCLVisualizer viewer0("Points removed from ikd-Tree");
    viewer0.addPointCloud<PointType>(src,src_color, "src");
    viewer0.addPointCloud<pcl::PointXYZRGB>(removed_colored, "removed");
    viewer0.setCameraPosition(-5, 30, 175,  0, 0, 0, 0.2, -1.0, 0.2);
    viewer0.setSize(1600, 900);

    pcl::visualization::PCLVisualizer viewer1("Map after Delete");
    viewer1.addPointCloud<PointType>(Remaining_Points,remaining_color, "remain");
    viewer1.setCameraPosition(-5, 30, 175,  0, 0, 0, 0.2, -1.0, 0.2);
    viewer1.setSize(1600, 900);

    while (!viewer0.wasStopped() && !viewer1.wasStopped()) {// } && !viewer2.wasStopped()) {
        viewer0.spinOnce();
        viewer1.spinOnce();
    }

    return 0;
}