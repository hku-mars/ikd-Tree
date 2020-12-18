#include "kd_tree.h"

/*
Description: K-D Tree improved for a better data structure performance. 
Author: Yixi Cai
email: yixicai@connect.hku.hk
*/

KD_TREE::KD_TREE(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.1, int max_point_num = 10) {
    delete_criterion_param = delete_param;
    balance_criterion_param = balance_param;
    Maximal_Point_Num = max_point_num;
    downsample_size = box_length;
    vector<Operation_Logger_Type> ().swap(Rebuild_Logger);            
    termination_flag = false;
    start_thread(); 
    // InitTreeNode(Root_Node);
}

KD_TREE::~KD_TREE()
{
    stop_thread();
    Delete_Storage_Disabled = true;
    delete_tree_nodes(&Root_Node, NOT_RECORD);
    PointVector ().swap(PCL_Storage);
    vector<Operation_Logger_Type> ().swap(Rebuild_Logger);
    printf("Max rebuild num is %d\n", max_rebuild_num);   
    printf("Max need rebuild num is %d\n", max_need_rebuild_num); 
}

void KD_TREE::Set_delete_criterion_param(float delete_param){
    delete_criterion_param = delete_param;
}

void KD_TREE::Set_balance_criterion_param(float balance_param){
    balance_criterion_param = balance_param;
}

void KD_TREE::InitTreeNode(KD_TREE_NODE * root){
    root->point.x = 0.0f;
    root->point.y = 0.0f;
    root->point.z = 0.0f;       
    root->node_range_x[0] = 0.0f;
    root->node_range_x[1] = 0.0f;
    root->node_range_y[0] = 0.0f;
    root->node_range_y[1] = 0.0f;    
    root->node_range_z[0] = 0.0f;
    root->node_range_z[1] = 0.0f;     
    root->division_axis = 0;
    root->father_ptr = nullptr;
    root->left_son_ptr = nullptr;
    root->right_son_ptr = nullptr;
    root->TreeSize = 0;
    root->invalid_point_num = 0;
    root->point_deleted = false;
    root->tree_deleted = false;
    root->point_downsample_deleted = false;
    root->leftson_search_flag = 0;    
    root->rightson_search_flag = 0;
    pthread_mutex_init(&(root->push_down_mutex_lock),NULL);
}   

int KD_TREE::size(){
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        return Root_Node->TreeSize;
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            return Root_Node->TreeSize;
            pthread_mutex_unlock(&working_flag_mutex);
        } else {
            return Treesize_tmp;
        }
    }
}

void KD_TREE::start_thread(){
    pthread_mutex_init(&termination_flag_mutex_lock, NULL);   
    pthread_mutex_init(&rebuild_ptr_mutex_lock, NULL);     
    pthread_mutex_init(&rebuild_logger_mutex_lock, NULL);    
    pthread_mutex_init(&points_deleted_rebuild_mutex_lock, NULL); 
    pthread_mutex_init(&working_flag_mutex, NULL);
    pthread_mutex_init(&search_flag_mutex, NULL);
    pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void*) this);
    printf("Multi thread started \n");    
}
void KD_TREE::stop_thread(){
    pthread_mutex_lock(&termination_flag_mutex_lock);
    termination_flag = true;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    if (rebuild_thread) pthread_join(rebuild_thread, NULL);
    pthread_mutex_destroy(&termination_flag_mutex_lock);
    pthread_mutex_destroy(&rebuild_logger_mutex_lock);
    pthread_mutex_destroy(&rebuild_ptr_mutex_lock);   
    pthread_mutex_destroy(&points_deleted_rebuild_mutex_lock);
    pthread_mutex_destroy(&working_flag_mutex);
    pthread_mutex_destroy(&search_flag_mutex);     
}

void * KD_TREE::multi_thread_ptr(void * arg){
    KD_TREE * handle = (KD_TREE*) arg;
    handle->multi_thread_rebuild();
}    

void KD_TREE::multi_thread_rebuild(){
    bool terminated = false;
    KD_TREE_NODE * father_ptr, ** new_node_ptr;
    pthread_mutex_lock(&termination_flag_mutex_lock);
    terminated = termination_flag;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    // Not sure whether we need a flag to notice this thread to finish and stop
    while (!terminated){
        pthread_mutex_lock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&working_flag_mutex);        
        if (Rebuild_Ptr != nullptr ){                    
            /* 
            // Start rebuilding the tree. Add, delete and delete box operations on this tree are blocked.
            // The blocked operations will be temporarily stored in the Rebuild_Logger
            */
            // printf("    ============= Tree size %d\n",(*Rebuild_Ptr)->TreeSize);
            max_rebuild_num = max(max_rebuild_num, (*Rebuild_Ptr)->TreeSize);
            if (*Rebuild_Ptr == Root_Node) Treesize_tmp = Root_Node->TreeSize;
            KD_TREE_NODE * old_root_node = (*Rebuild_Ptr);                            
            // printf("    =============   Rebuild size is %d\n", (*Rebuild_Ptr)->TreeSize);
            // printf("    =============   Is root %d\n",(*Rebuild_Ptr) == Root_Node);
            father_ptr = (*Rebuild_Ptr)->father_ptr;   
            PointVector ().swap(Rebuild_PCL_Storage);
            traverse_for_rebuild(*Rebuild_Ptr, Rebuild_PCL_Storage);                        
            KD_TREE_NODE * new_root_node = nullptr;            
            if (int(Rebuild_PCL_Storage.size()) > 0){               
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size()-1, Rebuild_PCL_Storage);             
                // Rebuild has been done. Updates the blocked operations into the new tree
                Operation_Logger_Type Operation;
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                while (!Rebuild_Logger.empty()){
                    Operation = Rebuild_Logger.back();
                    Rebuild_Logger.pop_back();
                    // printf("    Type %d\n", Operation.op);
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);                  
                    run_operation(&new_root_node, Operation);                   
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);               
                }   
               pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_lock(&search_flag_mutex);
            if (father_ptr->left_son_ptr == *Rebuild_Ptr) {
                father_ptr->left_son_ptr = new_root_node;
            } else if (father_ptr->right_son_ptr == *Rebuild_Ptr){             
                father_ptr->right_son_ptr = new_root_node;
            } else {
                printf("Error: Father ptr incompatible with current node\n");
            }
            if (new_root_node != nullptr) new_root_node->father_ptr = father_ptr;
            (*Rebuild_Ptr) = new_root_node;                      
            if (father_ptr == STATIC_ROOT_NODE) Root_Node = STATIC_ROOT_NODE->left_son_ptr;
            pthread_mutex_unlock(&search_flag_mutex);
            Rebuild_Ptr = nullptr;
            pthread_mutex_unlock(&working_flag_mutex);            
            delete_tree_nodes(&old_root_node, MULTI_THREAD_REC);
        } else {
            pthread_mutex_unlock(&working_flag_mutex);             
        }
        pthread_mutex_unlock(&rebuild_ptr_mutex_lock);         
        pthread_mutex_lock(&termination_flag_mutex_lock);
        terminated = termination_flag;
        pthread_mutex_unlock(&termination_flag_mutex_lock);          
        usleep(100); 
    }
    printf("Rebuild thread terminated normally\n");    
}

void KD_TREE::run_operation(KD_TREE_NODE ** root, Operation_Logger_Type operation){
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    float min_dist, tmp_dist;    
    switch (operation.op)
    {
    case ADD_POINT:      
        Add_by_point(root, operation.point, false);          
        break;
    case ADD_BOX:
        Add_by_range(root, operation.boxpoint, false);
        break;
    case DELETE_POINT:
        Delete_by_point(root, operation.point, false);
        break;
    case DELETE_BOX:
        Delete_by_range(root, operation.boxpoint, false, false);
        break;
    case DOWNSAMPLE_DELETE:
        Delete_by_range(root, operation.boxpoint, false, true);
        break;        
    default:
        break;
    }
}

void KD_TREE::Build(PointVector point_cloud){
    if (Root_Node != nullptr){
        delete_tree_nodes(&Root_Node, NOT_RECORD);
    }
    STATIC_ROOT_NODE = new KD_TREE_NODE;
    InitTreeNode(STATIC_ROOT_NODE); 
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size()-1, point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;
}

void KD_TREE::Nearest_Search(PointType point, int k_nearest, PointVector& Nearest_Points, vector<float> & Point_Distance){
    priority_queue<PointType_CMP> q; // Clear the priority queue;
    while (STATIC_ROOT_NODE->leftson_search_flag < 0) usleep(1);   
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        Search(Root_Node, k_nearest, point, q);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        Search(Root_Node, k_nearest, point, q);
        pthread_mutex_unlock(&search_flag_mutex);        
    }
    // printf("Search Counter is %d ",search_counter);
    int k_found = min(k_nearest,int(q.size()));
    PointVector ().swap(Nearest_Points);
    vector<float> Point_Distance;
    for (int i=0;i < k_found;i++){
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
    return;
}

void KD_TREE::Add_Points(PointVector & PointToAdd){
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    float min_dist, tmp_dist;
    for (int i=0; i<PointToAdd.size();i++){
        Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x/downsample_size)*downsample_size;
        Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0]+downsample_size;
        Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y/downsample_size)*downsample_size;
        Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1]+downsample_size; 
        Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z/downsample_size)*downsample_size;
        Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2]+downsample_size;   
        // printf("=================(%0.3f,%0.3f) (%0.3f,%0.3f) (%0.3f,%0.3f)\n",Box_of_Point.vertex_min[0],Box_of_Point.vertex_max[0],Box_of_Point.vertex_min[1],Box_of_Point.vertex_max[1],Box_of_Point.vertex_min[2],Box_of_Point.vertex_max[2]);
        mid_point.x = Box_of_Point.vertex_min[0] + (Box_of_Point.vertex_max[0]-Box_of_Point.vertex_min[0])/2.0;
        mid_point.y = Box_of_Point.vertex_min[1] + (Box_of_Point.vertex_max[1]-Box_of_Point.vertex_min[1])/2.0;
        mid_point.z = Box_of_Point.vertex_min[2] + (Box_of_Point.vertex_max[2]-Box_of_Point.vertex_min[2])/2.0;
        PointVector ().swap(Downsample_Storage);
        Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);
        min_dist = calc_dist(PointToAdd[i],mid_point);
        downsample_result = PointToAdd[i];                
        for (int index = 0; index < Downsample_Storage.size(); index++){
            tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
            if (tmp_dist < min_dist){
                min_dist = tmp_dist;
                downsample_result = Downsample_Storage[index];
            }
        }
        // printf("=================(%0.3f,%0.3f,%0.3f) (%0.3f,%0.3f,%0.3f) \n",downsample_result.x,downsample_result.y,downsample_result.z, mid_point.x, mid_point.y, mid_point.z);
        // printf("================= Downsample size is %d Mindist is %0.3f \n",int(Downsample_Storage.size()),sqrt(min_dist));
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
            if (DOWNSAMPLE_SWITCH){
                Delete_by_range(&Root_Node, Box_of_Point, true, true);
                Add_by_point(&Root_Node, downsample_result, true);  
            } else {        
                Add_by_point(&Root_Node, PointToAdd[i], true);        
            }
        } else {
            if (!pthread_mutex_trylock(&working_flag_mutex)){
                if (DOWNSAMPLE_SWITCH){
                    Delete_by_range(&Root_Node, Box_of_Point, true, true);
                    Add_by_point(&Root_Node, downsample_result, true);  
                } else {        
                    Add_by_point(&Root_Node, PointToAdd[i], true);        
                }
                pthread_mutex_unlock(&working_flag_mutex);               
            } else {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                if (DOWNSAMPLE_SWITCH){
                    Operation_Logger_Type  operation_delete;
                    operation_delete.boxpoint = Box_of_Point;
                    operation_delete.op = DOWNSAMPLE_DELETE; 
                    Rebuild_Logger.push_back(operation_delete);                   
                }
                Operation_Logger_Type operation;
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;                
                Rebuild_Logger.push_back(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                continue;
            }
        }
    }
    return;
}

void KD_TREE::Add_Point_Boxes(vector<BoxPointType> & BoxPoints){
    for (int i=0;i < BoxPoints.size();i++){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){                 
            Add_by_range(&Root_Node ,BoxPoints[i], true);
        } else {
            if (!pthread_mutex_trylock(&working_flag_mutex)){
                Add_by_range(&Root_Node ,BoxPoints[i], true);
                pthread_mutex_unlock(&working_flag_mutex);
            } else {
                Operation_Logger_Type operation;
                operation.boxpoint = BoxPoints[i];
                operation.op = ADD_BOX;
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push_back(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                continue;                
            }
        }        
    } 
    return;
}

void KD_TREE::Delete_Points(PointVector & PointToDel){   
    for (int i=0;i<PointToDel.size();i++){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){                 
            Delete_by_point(&Root_Node, PointToDel[i], true);
        } else {
            if (!pthread_mutex_trylock(&working_flag_mutex)){
                Delete_by_point(&Root_Node, PointToDel[i], true);
                pthread_mutex_unlock(&working_flag_mutex);
            } else {
                Operation_Logger_Type operation;
                operation.point = PointToDel[i];
                operation.op = DELETE_POINT;
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push_back(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                continue;                
            }
        }        
    }      
    return;
}

void KD_TREE::Delete_Point_Boxes(vector<BoxPointType> & BoxPoints){
    for (int i=0;i < BoxPoints.size();i++){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){                 
            Delete_by_range(&Root_Node ,BoxPoints[i], true, false);
        } else {
            if (!pthread_mutex_trylock(&working_flag_mutex)){
                Delete_by_range(&Root_Node ,BoxPoints[i], true, false);
                pthread_mutex_unlock(&working_flag_mutex);
            } else {
                Operation_Logger_Type operation;
                operation.boxpoint = BoxPoints[i];
                operation.op = DELETE_BOX;
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push_back(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                continue;                
            }
        }
    } 
    return;
}

void KD_TREE::acquire_removed_points(PointVector & removed_points){
    pthread_mutex_lock(&points_deleted_rebuild_mutex_lock); 
    for (int i = 0; i < Points_deleted.size();i++){
        removed_points.push_back(Points_deleted[i]);
    }
    for (int i = 0; i < Multithread_Points_deleted.size();i++){
        removed_points.push_back(Multithread_Points_deleted[i]);
    }    
    PointVector ().swap(Points_deleted);
    PointVector ().swap(Multithread_Points_deleted);
    pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);   
    return;
}

void KD_TREE::BuildTree(KD_TREE_NODE ** root, int l, int r, PointVector & Storage){
    if (l>r) return;
    *root = new KD_TREE_NODE;
    InitTreeNode(*root);
    int mid = (l+r)>>1; 
    // Find the best division Axis
    int i;
    float average[3] = {0,0,0};
    float covariance[3] = {0,0,0};
    for (i=l;i<=r;i++){
        average[0] += Storage[i].x;
        average[1] += Storage[i].y;
        average[2] += Storage[i].z;
    }
    for (i=0;i<3;i++) average[i] = average[i]/(r-l+1);
    for (i=l;i<=r;i++){
        covariance[0] += (Storage[i].x - average[0]) * (Storage[i].x - average[0]);
        covariance[1] += (Storage[i].y - average[1]) * (Storage[i].y - average[1]);  
        covariance[2] += (Storage[i].z - average[2]) * (Storage[i].z - average[2]);              
    }
    for (i=0;i<3;i++) covariance[i] = covariance[i]/(r-l+1);    
    int div_axis = 0;
    for (i = 1;i<3;i++){
        if (covariance[i] > covariance[div_axis]) div_axis = i;
    }
    (*root)->division_axis = div_axis;
    switch (div_axis)
    {
    case 0:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_x);
        break;
    case 1:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_y);
        break;
    case 2:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_z);
        break;
    default:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_x);
        break;
    }  
    (*root)->point = Storage[mid]; 
    KD_TREE_NODE * left_son = nullptr, * right_son = nullptr;
    BuildTree(&left_son, l, mid-1, Storage);
    BuildTree(&right_son, mid+1, r, Storage);  
    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;
    Update((*root));  
    return;
}

void KD_TREE::Rebuild(KD_TREE_NODE ** root){    
    KD_TREE_NODE * father_ptr;
    // Clear the PCL_Storage vector and release memory
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num) { //Multi_Thread_Rebuild_Minimal_Percent * Root_Node->TreeSize
        max_need_rebuild_num = max((*root)->TreeSize,max_need_rebuild_num);
        if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock)){     
            if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize)) {
                Rebuild_Ptr = root;          
            }
            pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        }
    } else {
        //printf("    Rebuild Tree size in main thread: %d\n", (*root)->TreeSize);
        father_ptr = (*root)->father_ptr;
        rebuild_counter += (*root)->TreeSize;
        int size_rec = (*root)->TreeSize;
        PCL_Storage.clear();
        traverse_for_rebuild(*root, PCL_Storage);       
        delete_tree_nodes(root, DELETE_POINTS_REC);
        BuildTree(root, 0, PCL_Storage.size()-1, PCL_Storage);
        if (*root != nullptr) (*root)->father_ptr = father_ptr;
    }
    // if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock)){     
    //     if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize)) Rebuild_Ptr = root;   
    //     pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
    // }    
    return;
}

void KD_TREE::Delete_by_range(KD_TREE_NODE ** root,  BoxPointType boxpoint, bool allow_rebuild, bool is_downsample){   
    if ((*root) == nullptr || (*root)->tree_deleted) return;
    Push_Down(*root);     
    if (boxpoint.vertex_max[0] + EPS < (*root)->node_range_x[0] || boxpoint.vertex_min[0] - EPS > (*root)->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] + EPS < (*root)->node_range_y[0] || boxpoint.vertex_min[1] - EPS > (*root)->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] + EPS < (*root)->node_range_z[0] || boxpoint.vertex_min[2] - EPS > (*root)->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] - EPS < (*root)->node_range_x[0] && boxpoint.vertex_max[0]+EPS > (*root)->node_range_x[1] && boxpoint.vertex_min[1]-EPS < (*root)->node_range_y[0] && boxpoint.vertex_max[1]+EPS > (*root)->node_range_y[1] && boxpoint.vertex_min[2]-EPS < (*root)->node_range_z[0] && boxpoint.vertex_max[2]+EPS > (*root)->node_range_z[1]){
        //printf("Delete in range (%0.3f,%0.3f) (%0.3f,%0.3f) (%0.3f,%0.3f)\n",x_range[0],x_range[1],y_range[0],y_range[1],z_range[0],z_range[1]);        
        //printf("Delete by range (%0.3f,%0.3f) (%0.3f,%0.3f) (%0.3f,%0.3f)\n",root->node_range_x[0],root->node_range_x[1],root->node_range_y[0],root->node_range_y[1],root->node_range_z[0],root->node_range_z[1]);
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->TreeSize;
        if (is_downsample){
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
        }
        return;
    }
    if (boxpoint.vertex_min[0]-EPS < (*root)->point.x && boxpoint.vertex_max[0]+EPS > (*root)->point.x && boxpoint.vertex_min[1]-EPS < (*root)->point.y && boxpoint.vertex_max[1]+EPS > (*root)->point.y && boxpoint.vertex_min[2]-EPS < (*root)->point.z && boxpoint.vertex_max[2]+EPS > (*root)->point.z){
        //printf("Deleted points: (%0.3f,%0.3f,%0.3f)\n",root->point.x,root->point.y,root->point.z);
        (*root)->point_deleted = true;
        if (is_downsample) (*root)->point_downsample_deleted = true;       
    }
    int left_lock_retval = -1, right_lock_retval = -1;
    Operation_Logger_Type delete_box_log;
    struct timespec Timeout;    
    if (is_downsample) delete_box_log.op = DOWNSAMPLE_DELETE;
        else delete_box_log.op = DELETE_BOX;
    delete_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
        Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } else {
        left_lock_retval = pthread_mutex_trylock(&working_flag_mutex);
        if (!left_lock_retval){
            Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
        } else {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push_back(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);            
        }
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){
        Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } else {
        right_lock_retval = pthread_mutex_trylock(&working_flag_mutex);
        if (!right_lock_retval){
            Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);         
        } else {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push_back(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                  
        }
    }    
    Update((*root));
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr;     
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if (!left_lock_retval || !right_lock_retval) pthread_mutex_unlock(&working_flag_mutex);    
    return;
}

void KD_TREE::Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild){   
    if ((*root) == nullptr || (*root)->tree_deleted) return;
    Push_Down((*root));
    if (same_point((*root)->point, point) && !(*root)->point_deleted) {          
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize) (*root)->tree_deleted = true;    
        return;
    }
    int retval = -1;
    Operation_Logger_Type delete_log;
    struct timespec Timeout;    
    delete_log.op = DELETE_POINT;
    delete_log.point = point;     
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z)){           
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){          
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);         
        } else {
            retval = pthread_mutex_trylock(&working_flag_mutex);
            if (!retval){        
                Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);         
            } else {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push_back(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
        }
    } else {       
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);         
        } else {
            retval = pthread_mutex_trylock(&working_flag_mutex);
            if (!retval){          
                Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);       
            } else {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push_back(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
        }        
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if (retval == 0) pthread_mutex_unlock(&working_flag_mutex);
    return;
}

void KD_TREE::Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild){
    if ((*root) == nullptr) return;
    Push_Down(*root);       
    if (boxpoint.vertex_max[0] + EPS < (*root)->node_range_x[0] || boxpoint.vertex_min[0] - EPS > (*root)->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] + EPS < (*root)->node_range_y[0] || boxpoint.vertex_min[1] - EPS > (*root)->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] + EPS < (*root)->node_range_z[0] || boxpoint.vertex_min[2] - EPS > (*root)->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] - EPS < (*root)->node_range_x[0] && boxpoint.vertex_max[0]+EPS > (*root)->node_range_x[1] && boxpoint.vertex_min[1]-EPS < (*root)->node_range_y[0] && boxpoint.vertex_max[1]+EPS > (*root)->node_range_y[1] && boxpoint.vertex_min[2]-EPS < (*root)->node_range_z[0] && boxpoint.vertex_max[2]+EPS > (*root)->node_range_z[1]){
        //printf("Delete in range (%0.3f,%0.3f) (%0.3f,%0.3f) (%0.3f,%0.3f)\n",x_range[0],x_range[1],y_range[0],y_range[1],z_range[0],z_range[1]);        
        //printf("Delete by range (%0.3f,%0.3f) (%0.3f,%0.3f) (%0.3f,%0.3f)\n",root->node_range_x[0],root->node_range_x[1],root->node_range_y[0],root->node_range_y[1],root->node_range_z[0],root->node_range_z[1]);
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = 0; 
        return;
    }
    if (boxpoint.vertex_min[0]-EPS < (*root)->point.x && boxpoint.vertex_max[0]+EPS > (*root)->point.x && boxpoint.vertex_min[1]-EPS < (*root)->point.y && boxpoint.vertex_max[1]+EPS > (*root)->point.y && boxpoint.vertex_min[2]-EPS < (*root)->point.z && boxpoint.vertex_max[2]+EPS > (*root)->point.z){
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }
    int left_lock_retval = -1, right_lock_retval = -1;
    Operation_Logger_Type add_box_log;
    struct timespec Timeout;    
    add_box_log.op = ADD_BOX;
    add_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
        Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    } else {
        left_lock_retval = pthread_mutex_trylock(&working_flag_mutex);
        if (!left_lock_retval){
            Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
        } else {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push_back(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);            
        }
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){
        Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    } else {
        right_lock_retval = pthread_mutex_trylock(&working_flag_mutex);
        if (!right_lock_retval){
            Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
            pthread_mutex_unlock(&working_flag_mutex);           
        } else {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push_back(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                  
        }
    }
    Update((*root));
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if (!left_lock_retval || !right_lock_retval) pthread_mutex_unlock(&working_flag_mutex);
    return;
}

void KD_TREE::Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild){     
    if (*root == nullptr){
        *root = new KD_TREE_NODE;
        InitTreeNode(*root);
        (*root)->point = point;
        Update(*root);
        return;
    }           
    int retval = -1;
    Operation_Logger_Type add_log;
    struct timespec Timeout;    
    add_log.op = ADD_POINT;
    add_log.point = point; 
    Push_Down(*root);      
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z)){
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){          
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild);
        } else {
            retval = pthread_mutex_trylock(&working_flag_mutex);
            if (!retval){        
                Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild);
            } else {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push_back(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
        }
    } else {  
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
            Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild);
        } else {
            retval = pthread_mutex_trylock(&working_flag_mutex);
            if (!retval){          
                Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild);       

            } else {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push_back(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
        }
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if (!retval) pthread_mutex_unlock(&working_flag_mutex);        
    return;
}

void KD_TREE::Search(KD_TREE_NODE * root, int k_nearest, PointType point, priority_queue<PointType_CMP> &q){
    if (root == nullptr || root->tree_deleted) return;   
    int retval; 
    if (root->need_push_down_to_left || root->need_push_down_to_right) {
        retval = pthread_mutex_trylock(&(root->push_down_mutex_lock));
        if (retval == 0){
            Push_Down(root);
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        } else {
            pthread_mutex_lock(&(root->push_down_mutex_lock));
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
    }
    if (!root->point_deleted){
        float dist = calc_dist(point, root->point);
        if (q.size() < k_nearest || dist < q.top().dist){
            if (q.size() >= k_nearest) q.pop();
            PointType_CMP current_point{root->point, dist};                    
            q.push(current_point);            
        }
    }  
    
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    if (q.size()< k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist){
        if (dist_left_node <= dist_right_node) {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                Search(root->left_son_ptr, k_nearest, point,q);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point,q);  
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist) {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                    Search(root->right_son_ptr, k_nearest, point,q);                       
                } else {
                    pthread_mutex_lock(&search_flag_mutex);
                    Search(root->right_son_ptr, k_nearest, point,q);  
                    pthread_mutex_unlock(&search_flag_mutex);
                }                
            }
        } else {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                Search(root->right_son_ptr, k_nearest, point,q);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point,q);  
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist) {            
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                    Search(root->left_son_ptr, k_nearest, point,q);                       
                } else {
                    pthread_mutex_lock(&search_flag_mutex);
                    Search(root->left_son_ptr, k_nearest, point,q);  
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
    } else {
        if (dist_left_node < q.top().dist) {        
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                Search(root->left_son_ptr, k_nearest, point,q);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point,q);  
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
        if (dist_right_node < q.top().dist) {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                Search(root->right_son_ptr, k_nearest, point,q);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point,q);  
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    }  
    return;
}

void KD_TREE::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector & Storage){
    if (root == nullptr) return;
    Push_Down(root);       
    if (boxpoint.vertex_max[0] + EPS < root->node_range_x[0] || boxpoint.vertex_min[0] - EPS > root->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] + EPS < root->node_range_y[0] || boxpoint.vertex_min[1] - EPS > root->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] + EPS < root->node_range_z[0] || boxpoint.vertex_min[2] - EPS > root->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] - EPS < root->node_range_x[0] && boxpoint.vertex_max[0]+EPS > root->node_range_x[1] && boxpoint.vertex_min[1]-EPS < root->node_range_y[0] && boxpoint.vertex_max[1]+EPS > root->node_range_y[1] && boxpoint.vertex_min[2]-EPS < root->node_range_z[0] && boxpoint.vertex_max[2]+EPS > root->node_range_z[1]){
        traverse_for_rebuild(root, Storage);
        return;
    }
    if (boxpoint.vertex_min[0]-EPS < root->point.x && boxpoint.vertex_max[0]+EPS > root->point.x && boxpoint.vertex_min[1]-EPS < root->point.y && boxpoint.vertex_max[1]+EPS > root->point.y && boxpoint.vertex_min[2]-EPS < root->point.z && boxpoint.vertex_max[2]+EPS > root->point.z){
        if (!root->point_deleted) Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr){
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr){
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    return;    
}

bool KD_TREE::Criterion_Check(KD_TREE_NODE * root){
    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size){
        return false;
    }
    float balance_evaluation = 0.0f;
    float delete_evaluation = 0.0f;
    KD_TREE_NODE * son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
    delete_evaluation = float(root->invalid_point_num)/ root->TreeSize;
    balance_evaluation = float(son_ptr->TreeSize) / root->TreeSize;
    if (delete_evaluation > delete_criterion_param){
        return true;
    }
    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1-balance_criterion_param){
        return true;
    } 
    return false;
}

void KD_TREE::Push_Down(KD_TREE_NODE *root){
    if (root == nullptr) return;
    if (root->need_push_down_to_left && root->left_son_ptr != nullptr){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_left = false;                
        } else {
            if (!pthread_mutex_trylock(&working_flag_mutex)){
                root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
                root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
                root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
                root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
                root->left_son_ptr->need_push_down_to_left = true;
                root->left_son_ptr->need_push_down_to_right = true;
                root->need_push_down_to_left = false;
                pthread_mutex_unlock(&working_flag_mutex);            
            }
        }
    }
    if (root->need_push_down_to_right && root->right_son_ptr != nullptr){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_right = false;
        } else {
            if (!pthread_mutex_trylock(&working_flag_mutex)){
                root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
                root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
                root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
                root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
                root->right_son_ptr->need_push_down_to_left = true;
                root->right_son_ptr->need_push_down_to_right = true;
                root->need_push_down_to_right = false;
                pthread_mutex_unlock(&working_flag_mutex);
            }
        }
    }
    return;
}

void KD_TREE::Update(KD_TREE_NODE * root){
    KD_TREE_NODE * left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE * right_son_ptr = root->right_son_ptr;
    // Update Tree Size
    if (left_son_ptr != nullptr && right_son_ptr != nullptr){
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + (root->point_deleted? 1:0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;
        root->node_range_x[0] = min(min(left_son_ptr->node_range_x[0],right_son_ptr->node_range_x[0]),root->point.x);
        root->node_range_x[1] = max(max(left_son_ptr->node_range_x[1],right_son_ptr->node_range_x[1]),root->point.x);
        root->node_range_y[0] = min(min(left_son_ptr->node_range_y[0],right_son_ptr->node_range_y[0]),root->point.y);
        root->node_range_y[1] = max(max(left_son_ptr->node_range_y[1],right_son_ptr->node_range_y[1]),root->point.y);        
        root->node_range_z[0] = min(min(left_son_ptr->node_range_z[0],right_son_ptr->node_range_z[0]),root->point.z);
        root->node_range_z[1] = max(max(left_son_ptr->node_range_z[1],right_son_ptr->node_range_z[1]),root->point.z);         
    } else if (left_son_ptr != nullptr){
        root->TreeSize = left_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + (root->point_deleted?1:0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        root->node_range_x[0] = min(left_son_ptr->node_range_x[0],root->point.x);
        root->node_range_x[1] = max(left_son_ptr->node_range_x[1],root->point.x);
        root->node_range_y[0] = min(left_son_ptr->node_range_y[0],root->point.y);
        root->node_range_y[1] = max(left_son_ptr->node_range_y[1],root->point.y); 
        root->node_range_z[0] = min(left_son_ptr->node_range_z[0],root->point.z);
        root->node_range_z[1] = max(left_son_ptr->node_range_z[1],root->point.z);               
    } else if (right_son_ptr != nullptr){
        root->TreeSize = right_son_ptr->TreeSize + 1;
        root->invalid_point_num = right_son_ptr->invalid_point_num + (root->point_deleted? 1:0);
        root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;        
        root->node_range_x[0] = min(right_son_ptr->node_range_x[0],root->point.x);
        root->node_range_x[1] = max(right_son_ptr->node_range_x[1],root->point.x);
        root->node_range_y[0] = min(right_son_ptr->node_range_y[0],root->point.y);
        root->node_range_y[1] = max(right_son_ptr->node_range_y[1],root->point.y); 
        root->node_range_z[0] = min(right_son_ptr->node_range_z[0],root->point.z);
        root->node_range_z[1] = max(right_son_ptr->node_range_z[1],root->point.z);        
    } else {
        root->TreeSize = 1;
        root->invalid_point_num = (root->point_deleted? 1:0);
        root->tree_downsample_deleted = root->point_downsample_deleted;
        root->tree_deleted = root->point_deleted;
        root->node_range_x[0] = root->point.x;
        root->node_range_x[1] = root->point.x;        
        root->node_range_y[0] = root->point.y;
        root->node_range_y[1] = root->point.y; 
        root->node_range_z[0] = root->point.z;
        root->node_range_z[1] = root->point.z;                 
    }
    if (left_son_ptr != nullptr) left_son_ptr -> father_ptr = root;
    if (right_son_ptr != nullptr) right_son_ptr -> father_ptr = root;
    return;
}

void KD_TREE::traverse_for_rebuild(KD_TREE_NODE * root, PointVector &Storage){
    if (root == nullptr || root->tree_deleted) return;
    Push_Down(root);
    if (!root->point_deleted) {
        Storage.push_back(root->point);
    }
    traverse_for_rebuild(root->left_son_ptr, Storage);
    traverse_for_rebuild(root->right_son_ptr, Storage);
    return;
}

void KD_TREE::delete_tree_nodes(KD_TREE_NODE ** root, delete_point_storage_set storage_type){ 
    if (*root == nullptr) return;
    Push_Down(*root);    
    delete_tree_nodes(&(*root)->left_son_ptr, storage_type);
    delete_tree_nodes(&(*root)->right_son_ptr, storage_type);  
    switch (storage_type)
    {
    case NOT_RECORD:
        break;
    case DELETE_POINTS_REC:
        // printf("------Deleteing Node: (%0.3f, %0.3f, %0.3f)----\n", (*root)->point.x, (*root)->point.y, (*root)->point.z); 
        if ((*root)->point_deleted && !(*root)->point_downsample_deleted) {
            Points_deleted.push_back((*root)->point);
        }
        if (Rebuild_Ptr != nullptr && *root == *Rebuild_Ptr){
            printf("\n\n\n--------------BIG ERROR!!!! \n");
            printf("%d\n\n\n",(*Rebuild_Ptr)->TreeSize);
        }
        // printf("------Finish Delete----------------------------\n");              
        break;
    case MULTI_THREAD_REC:
        // printf("    ------Deleteing Node: (%0.3f, %0.3f, %0.3f)----\n", (*root)->point.x, (*root)->point.y, (*root)->point.z);    
        pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);    
        if ((*root)->point_deleted  && !(*root)->point_downsample_deleted) {
            Multithread_Points_deleted.push_back((*root)->point);
        }
        pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);     
        // printf("    ------Finish Delete----------------------------\n");  
        break;
    case DOWNSAMPLE_REC:
        if (!(*root)->point_deleted) Downsample_Storage.push_back((*root)->point);
        break;
    default:
        break;
    }               
    delete *root;
    *root = nullptr;                    

    return;
}

bool KD_TREE::same_point(PointType a, PointType b){
    return (fabs(a.x-b.x) < EPS && fabs(a.y-b.y) < EPS && fabs(a.z-b.z) < EPS );
}

float KD_TREE::calc_dist(PointType a, PointType b){
    float dist = 0.0f;
    dist = (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z);
    return dist;
}

float KD_TREE::calc_box_dist(KD_TREE_NODE * node, PointType point){
    if (node == nullptr) return INFINITY;
    float min_dist = 0.0;
    if (point.x < node->node_range_x[0]) min_dist += (point.x - node->node_range_x[0])*(point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1]) min_dist += (point.x - node->node_range_x[1])*(point.x - node->node_range_x[1]);
    if (point.y < node->node_range_y[0]) min_dist += (point.y - node->node_range_y[0])*(point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1]) min_dist += (point.y - node->node_range_y[1])*(point.y - node->node_range_y[1]);
    if (point.z < node->node_range_z[0]) min_dist += (point.z - node->node_range_z[0])*(point.z - node->node_range_z[0]);
    if (point.z > node->node_range_z[1]) min_dist += (point.z - node->node_range_z[1])*(point.z - node->node_range_z[1]);
    return min_dist;
}

bool KD_TREE::point_cmp_x(PointType a, PointType b) { return a.x < b.x;}
bool KD_TREE::point_cmp_y(PointType a, PointType b) { return a.y < b.y;}
bool KD_TREE::point_cmp_z(PointType a, PointType b) { return a.z < b.z;}