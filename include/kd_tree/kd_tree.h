#pragma once
#include <pcl/point_types.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <stdio.h>
#include <queue>
#include <pthread.h>
#include <chrono>
#include <time.h>

#define EPS 1e-6
#define Minimal_Unbalanced_Tree_Size 10 
#define Multi_Thread_Rebuild_Minimal_Percent 0.2
#define Multi_Thread_Rebuild_Point_Num 200000
#define DOWNSAMPLE_SWITCH true

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

struct KD_TREE_NODE
{
    PointType point;
    int division_axis;  
    int TreeSize = 1;
    int invalid_point_num = 0;
    bool point_deleted = false;
    bool tree_deleted = false; 
    bool point_downsample_deleted = false;
    bool tree_downsample_deleted = false;
    bool need_push_down_to_left = false;
    bool need_push_down_to_right = false;
    pthread_mutex_t push_down_mutex_lock;
    float node_range_x[2], node_range_y[2], node_range_z[2];   
    KD_TREE_NODE *left_son_ptr = nullptr;
    KD_TREE_NODE *right_son_ptr = nullptr;
    KD_TREE_NODE *father_ptr = nullptr;
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
    float vertex_min[3];
    float vertex_max[3];
};


enum operation_set {ADD_POINT, DELETE_POINT, DELETE_BOX, ADD_BOX, DOWNSAMPLE_DELETE};

enum delete_point_storage_set {NOT_RECORD, DELETE_POINTS_REC, MULTI_THREAD_REC, DOWNSAMPLE_REC};

struct Operation_Logger_Type{
    PointType point;
    BoxPointType boxpoint;
    operation_set op;
};


class KD_TREE
{
private:
    // debug
    int add_counter;
    int delete_counter;
    int max_rebuild_num = 0;
    int max_need_rebuild_num = 0;
    // Multi-thread Tree Rebuild
    bool termination_flag = false;
    pthread_t rebuild_thread;
    pthread_mutex_t termination_flag_mutex_lock, rebuild_ptr_mutex_lock, working_flag_mutex, search_flag_mutex;
    pthread_mutex_t rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock;
    vector<Operation_Logger_Type> Rebuild_Logger;
    PointVector Rebuild_PCL_Storage;
    KD_TREE_NODE ** Rebuild_Ptr;
    int search_mutex_counter = 0;
    static void * multi_thread_ptr(void *arg);
    void multi_thread_rebuild();
    void start_thread();
    void stop_thread();
    void run_operation(KD_TREE_NODE ** root, Operation_Logger_Type operation);
    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_size = 0.2f;   
    float Maximal_Point_Num = 10;
    bool Delete_Storage_Disabled = false;
    KD_TREE_NODE * STATIC_ROOT_NODE = nullptr;
    PointVector Points_deleted;
    PointVector Downsample_Storage;
    PointVector Multithread_Points_deleted;
    void InitTreeNode(KD_TREE_NODE * root);
    void Test_Lock_States(KD_TREE_NODE *root);
    void BuildTree(KD_TREE_NODE ** root, int l, int r, PointVector & Storage);
    void Rebuild(KD_TREE_NODE ** root);
    void Delete_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);
    void Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild);
    void Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild);
    void Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild);
    void Search(KD_TREE_NODE * root, int k_nearest, PointType point, priority_queue<PointType_CMP> &q);
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage);
    bool Criterion_Check(KD_TREE_NODE * root);
    void Push_Down(KD_TREE_NODE * root);
    void Update(KD_TREE_NODE * root); 
    void delete_tree_nodes(KD_TREE_NODE ** root, delete_point_storage_set storage_type);
    void downsample(KD_TREE_NODE ** root);
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
    int size();
    void Build(PointVector point_cloud);
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> & Point_Distance);
    void Add_Points(PointVector & PointToAdd);
    void Add_Point_Boxes(vector<BoxPointType> & BoxPoints);
    void Delete_Points(PointVector & PointToDel);
    void Delete_Point_Boxes(vector<BoxPointType> & BoxPoints);
    void traverse_for_rebuild(KD_TREE_NODE * root, PointVector &Storage);
    void acquire_removed_points(PointVector & removed_points);
    PointVector PCL_Storage;     
    KD_TREE_NODE * Root_Node = nullptr;  
    vector<float> add_rec,delete_rec;
    vector<int>   add_counter_rec, delete_counter_rec;
    int rebuild_counter = 0;
    // void Compatibility_Check();
};
