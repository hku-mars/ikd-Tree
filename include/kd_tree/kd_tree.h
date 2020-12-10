#pragma once
#include <pcl/point_types.h>
#include <stdio.h>
#include <queue>

#define EPS 1e-6
#define Minimal_Unbalanced_Tree_Size 200


using namespace std;

typedef pcl::PointXYZINormal PointType;

struct KD_TREE_NODE
{
    PointType point;
    int division_axis;  
    int TreeSize = 1;
    int invalid_point_num = 0;
    bool point_deleted = false;
    bool tree_deleted = false; 
    bool need_rebuild = false;
    float node_range_x[2], node_range_y[2], node_range_z[2];   
    KD_TREE_NODE *left_son_ptr = nullptr;
    KD_TREE_NODE *right_son_ptr = nullptr;
};

struct PointType_CMP{
    PointType point;
    float dist;
    PointType_CMP (PointType p, float d){
        this->point = p;
        this->dist = d;
    };
    bool operator < (const PointType_CMP &a)const{
        return dist < a.dist;
    }
};

struct BoxPointType{
    int vertex_min[3];
    int vertex_max[3];
};

class KD_TREE
{
private:
    int search_counter = 0;
    int delete_counter = 0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_volume = 0.1*0.1*0.1;
    float Maximal_Point_Num = 10;
    bool downsample_flag = false;
    priority_queue<PointType_CMP> q;
    vector<PointType> Points_deleted;
    void BuildTree(KD_TREE_NODE * &root, int l, int r);
    void Rebuild(KD_TREE_NODE * &root);
    void Delete_by_range(KD_TREE_NODE * root, BoxPointType boxpoint);
    bool Delete_by_point(KD_TREE_NODE * root, PointType point);
    void Add(KD_TREE_NODE * &root, PointType point);
    void Search(KD_TREE_NODE * root, int k_nearest, PointType point);
    bool Criterion_Check(KD_TREE_NODE * root);
    void Push_Down(KD_TREE_NODE * root);
    void Update(KD_TREE_NODE * root); 
    void delete_tree_nodes(KD_TREE_NODE * &root);
    void downsample(KD_TREE_NODE * &root);
    bool same_point(PointType a, PointType b);
    float calc_dist(PointType a, PointType b);
    float calc_box_dist(KD_TREE_NODE * node, PointType point);
    static bool point_cmp_x(PointType a, PointType b); 
    static bool point_cmp_y(PointType a, PointType b); 
    static bool point_cmp_z(PointType a, PointType b); 

public:
    KD_TREE(float, float, float, int);
    ~KD_TREE();
    void Set_delete_criterion_param(float delete_param);
    void Set_balance_criterion_param(float balance_param);
    void set_downsample_param(float box_length, int Maximal_Point_Num);
    void Build(vector<PointType> point_cloud);
    void Nearest_Search(PointType point, int k_nearest, vector<PointType> &Nearest_Points);
    void Add_Points(vector<PointType> & PointToAdd);
    void Delete_Points(vector<PointType> & PointToDel);
    void Delete_Point_Boxes(vector<BoxPointType> & BoxPoints);
    void traverse_for_rebuild(KD_TREE_NODE * root);
    void acquire_removed_points(vector<PointType> & removed_points);
    vector<PointType> PCL_Storage;     
       
    KD_TREE_NODE * Root_Node = nullptr;    
    int rebuild_counter = 0;
    // void Compatibility_Check();
};
