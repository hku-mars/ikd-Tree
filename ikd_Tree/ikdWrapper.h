#pragma once
#include "ikd_Tree.h"
#include <pcl/search/search.h>

template <typename PointT>
class IKDTree : public pcl::search::Search<PointT>
{

public:
    using PointCloud = typename pcl::search::Search<PointT>::PointCloud;
    using PointCloudConstPtr = typename pcl::search::Search<PointT>::PointCloudConstPtr;

    IKDTree(float delete_param = 0.5, float balance_param = 0.6, float box_length = 0.2) 
        : tree_(delete_param, balance_param, box_length)
    {
    }

    // TODO override PCL
    // int nearestKSearch(const PointT &point, int k, std::vector<int> &k_indices,
    //                    std::vector<float> &k_sqr_distances) override
    // {
    // }

private:
    KD_TREE<PointT> tree_;


    
};