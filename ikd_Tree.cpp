#include "ikd_Tree.h"

/*
Description: ikd-Tree: an incremental k-d tree for robotic applications 
Author: Yixi Cai
email: yixicai@connect.hku.hk
*/

KD_TREE::KD_TREE(float delete_param, float balance_param, float box_length) {
    delete_criterion_param = delete_param;
    balance_criterion_param = balance_param;
    downsample_size = box_length;
    queue<Operation_Logger_Type> ().swap(Rebuild_Logger);            
    termination_flag = false;
    start_thread(); 
}

KD_TREE::~KD_TREE()
{
    stop_thread();
    Delete_Storage_Disabled = true;
    delete_tree_nodes(&Root_Node, NOT_RECORD);
    PointVector ().swap(PCL_Storage);
    queue<Operation_Logger_Type> ().swap(Rebuild_Logger); 
}

void KD_TREE::Set_delete_criterion_param(float delete_param){
    delete_criterion_param = delete_param;
}

void KD_TREE::Set_balance_criterion_param(float balance_param){
    balance_criterion_param = balance_param;
}

void KD_TREE::set_downsample_param(float downsample_param){
    downsample_size = downsample_param;
}

void KD_TREE::InitializeKDTree(float delete_param, float balance_param, float box_length){
    Set_delete_criterion_param(delete_param);
    Set_balance_criterion_param(balance_param);
    set_downsample_param(box_length);
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
    root->need_push_down_to_left = false;
    root->need_push_down_to_right = false;
    root->point_downsample_deleted = false;
    root->alpha_bal = 0.5;
    root->alpha_del = 0.0;
    pthread_mutex_init(&(root->push_down_mutex_lock),NULL);
}   

int KD_TREE::size(){
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        if (Root_Node != nullptr) {
            return Root_Node->TreeSize;
        } else {
            return 0;
        }
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            s = Root_Node->TreeSize;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        } else {
            return Treesize_tmp;
        }
    }
}

BoxPointType KD_TREE::tree_range(){
    BoxPointType range;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        if (Root_Node != nullptr) {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
        } else {
            memset(&range, 0, sizeof(range));
        }
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
            pthread_mutex_unlock(&working_flag_mutex);
        } else {
            memset(&range, 0, sizeof(range));
        }
    }
    return range;
}

int KD_TREE::validnum(){
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        return (Root_Node->TreeSize - Root_Node->invalid_point_num);
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            s = Root_Node->TreeSize-Root_Node->invalid_point_num;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        } else {
            return Validnum_tmp;
        }
    }
}

void KD_TREE::root_alpha(float &alpha_bal, float &alpha_del){
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        alpha_bal = Root_Node->alpha_bal;
        alpha_del = Root_Node->alpha_del;
        return;
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            alpha_bal = Root_Node->alpha_bal;
            alpha_del = Root_Node->alpha_del;
            pthread_mutex_unlock(&working_flag_mutex);
            return;
        } else {
            alpha_bal = alpha_bal_tmp;
            alpha_del = alpha_del_tmp;      
            return;
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
    return nullptr;
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
            /* Traverse and copy */
            if (Rebuild_Logger.size()>0){
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }
            rebuild_flag = true;
            max_rebuild_num = max(max_rebuild_num, (*Rebuild_Ptr)->TreeSize);
            if (*Rebuild_Ptr == Root_Node) {
                Treesize_tmp = Root_Node->TreeSize;
                Validnum_tmp = Root_Node->TreeSize - Root_Node->invalid_point_num;
                alpha_bal_tmp = Root_Node->alpha_bal;
                alpha_del_tmp = Root_Node->alpha_del;
            }
            KD_TREE_NODE * old_root_node = (*Rebuild_Ptr);                            
            father_ptr = (*Rebuild_Ptr)->father_ptr;  
            PointVector ().swap(Rebuild_PCL_Storage);
            flatten(*Rebuild_Ptr, Rebuild_PCL_Storage); 
            pthread_mutex_unlock(&working_flag_mutex);   
            /* Rebuild and update missed operations*/
            Operation_Logger_Type Operation;
            KD_TREE_NODE * new_root_node = nullptr;            
            if (int(Rebuild_PCL_Storage.size()) > 0){               
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size()-1, Rebuild_PCL_Storage);             
                // Rebuild has been done. Updates the blocked operations into the new tree  
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                while (!Rebuild_Logger.empty()){
                    Operation = Rebuild_Logger.front();
                    Rebuild_Logger.pop();
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);                  
                    run_operation(&new_root_node, Operation);                                       
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);               
                }   
               pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }  
            /* Replace to original tree*/          
            pthread_mutex_lock(&working_flag_mutex);
            if (Drop_MultiThread_Rebuild){
                delete_tree_nodes(&new_root_node, NOT_RECORD);
                rebuild_flag = false;   
                Rebuild_Ptr = nullptr;
                Drop_MultiThread_Rebuild = false;
                delete_tree_nodes(&new_root_node, NOT_RECORD);
                pthread_mutex_unlock(&working_flag_mutex);
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter != 0){
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(10);             
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter = -1;
                pthread_mutex_unlock(&search_flag_mutex);
                if (father_ptr->left_son_ptr == *Rebuild_Ptr) {
                    father_ptr->left_son_ptr = new_root_node;
                } else if (father_ptr->right_son_ptr == *Rebuild_Ptr){             
                    father_ptr->right_son_ptr = new_root_node;
                } else {
                    throw "Error: Father ptr incompatible with current node\n";
                }
                if (new_root_node != nullptr) new_root_node->father_ptr = father_ptr;
                (*Rebuild_Ptr) = new_root_node;                 
                if (father_ptr == STATIC_ROOT_NODE) Root_Node = STATIC_ROOT_NODE->left_son_ptr;             
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter = 0;
                pthread_mutex_unlock(&search_flag_mutex);
                Rebuild_Ptr = nullptr;
                pthread_mutex_unlock(&working_flag_mutex);
                rebuild_flag = false;                     
                /* Delete discarded tree nodes */  
                delete_tree_nodes(&old_root_node, MULTI_THREAD_REC);
            }
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
    case PUSH_DOWN:
        (*root)->tree_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->point_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->tree_deleted = operation.tree_deleted || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = (*root)->tree_deleted || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;     
        break;
    default:
        break;
    }
}

void KD_TREE::Build(PointVector point_cloud){
    if (Root_Node != nullptr){
        delete_tree_nodes(&Root_Node, NOT_RECORD);
    }
    if (point_cloud.size() == 0) return;
    STATIC_ROOT_NODE = new KD_TREE_NODE;
    InitTreeNode(STATIC_ROOT_NODE); 
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size()-1, point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;    
}

void KD_TREE::Nearest_Search(PointType point, int k_nearest, PointVector& Nearest_Points, vector<float> & Point_Distance){   
    priority_queue<PointType_CMP> q; // Clear the priority queue;
    PointVector ().swap(Nearest_Points);
    vector<float> ().swap(Point_Distance);
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        Search(Root_Node, k_nearest, point, q);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        while (search_mutex_counter == -1)
        {
            pthread_mutex_unlock(&search_flag_mutex);
            usleep(1);
            pthread_mutex_lock(&search_flag_mutex);
        }
        search_mutex_counter += 1;
        pthread_mutex_unlock(&search_flag_mutex);  
        Search(Root_Node, k_nearest, point, q);  
        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;
        pthread_mutex_unlock(&search_flag_mutex);      
    }
    int k_found = min(k_nearest,int(q.size()));
    PointVector ().swap(Nearest_Points);
    vector<float> ().swap(Point_Distance);
    for (int i=0;i < k_found;i++){
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
    return;
}

void KD_TREE::Add_Points(PointVector & PointToAdd, bool downsample_on){
    int NewPointSize = PointToAdd.size();
    int tree_size = size();
    if (tree_size>0 && NewPointSize > Multi_Thread_Rebuild_Point_Num && float(NewPointSize)/float(tree_size) > ForceRebuildPercentage){
        pthread_mutex_lock(&working_flag_mutex);
        Drop_MultiThread_Rebuild = true;
        Rebuild_Ptr = nullptr;
        PointVector ().swap(PCL_Storage);        
        queue<Operation_Logger_Type> ().swap(Rebuild_Logger);
        flatten(Root_Node, PCL_Storage);
        PCL_Storage.insert(PCL_Storage.end(), PointToAdd.begin(),PointToAdd.end());
        Build(PCL_Storage);
        pthread_mutex_unlock(&working_flag_mutex);
        return;
    }
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    float min_dist, tmp_dist;
    for (int i=0; i<PointToAdd.size();i++){
        if (downsample_switch){
            Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0]+downsample_size;
            Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1]+downsample_size; 
            Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2]+downsample_size;   
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
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){  
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)){
                    Delete_by_range(&Root_Node, Box_of_Point, true, true);     
                    Add_by_point(&Root_Node, downsample_result, true);                      
                }
            } else {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)){
                    Operation_Logger_Type  operation_delete, operation;
                    operation_delete.boxpoint = Box_of_Point;
                    operation_delete.op = DOWNSAMPLE_DELETE;
                    operation.point = downsample_result;
                    operation.op = ADD_POINT;
                    pthread_mutex_lock(&working_flag_mutex);
                    Delete_by_range(&Root_Node, Box_of_Point, false , true);                 
                    Add_by_point(&Root_Node, downsample_result, false);
                    if (rebuild_flag){
                        pthread_mutex_lock(&rebuild_logger_mutex_lock);
                        Rebuild_Logger.push(operation_delete);
                        Rebuild_Logger.push(operation);
                        pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    }
                    pthread_mutex_unlock(&working_flag_mutex);
                }
            }
        } else {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
                Add_by_point(&Root_Node, PointToAdd[i], true);     
            } else {
                Operation_Logger_Type operation;
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;                
                pthread_mutex_lock(&working_flag_mutex);
                Add_by_point(&Root_Node, PointToAdd[i], false);
                if (rebuild_flag){
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                    Rebuild_Logger.push(operation);
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&working_flag_mutex);       
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
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = ADD_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_range(&Root_Node ,BoxPoints[i], false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }               
            pthread_mutex_unlock(&working_flag_mutex);
        }    
    } 
    return;
}

void KD_TREE::Delete_Points(PointVector & PointToDel){        
    for (int i=0;i<PointToDel.size();i++){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){               
            Delete_by_point(&Root_Node, PointToDel[i], true);
        } else {
            Operation_Logger_Type operation;
            operation.point = PointToDel[i];
            operation.op = DELETE_POINT;
            pthread_mutex_lock(&working_flag_mutex);        
            Delete_by_point(&Root_Node, PointToDel[i], false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }      
    }      
    return;
}

void KD_TREE::Delete_Point_Boxes(vector<BoxPointType> & BoxPoints){      
    for (int i=0;i < BoxPoints.size();i++){ 
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){               
            Delete_by_range(&Root_Node ,BoxPoints[i], true, false);
        } else {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = DELETE_BOX;     
            pthread_mutex_lock(&working_flag_mutex); 
            Delete_by_range(&Root_Node ,BoxPoints[i], false, false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }                
            pthread_mutex_unlock(&working_flag_mutex);
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
    Points_deleted.clear();
    Multithread_Points_deleted.clear();
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
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num) { 
        max_need_rebuild_num = max((*root)->TreeSize,max_need_rebuild_num);
        if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock)){     
            if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize)) {
                Rebuild_Ptr = root;          
            }
            pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        }
    } else {
        father_ptr = (*root)->father_ptr;
        rebuild_counter += (*root)->TreeSize;
        int size_rec = (*root)->TreeSize;
        PCL_Storage.clear();
        flatten(*root, PCL_Storage);       
        delete_tree_nodes(root, DELETE_POINTS_REC);
        BuildTree(root, 0, PCL_Storage.size()-1, PCL_Storage);
        if (*root != nullptr) (*root)->father_ptr = father_ptr;
        if (*root == Root_Node) STATIC_ROOT_NODE->left_son_ptr = *root;
    } 
    return;
}

void KD_TREE::Delete_by_range(KD_TREE_NODE ** root,  BoxPointType boxpoint, bool allow_rebuild, bool is_downsample){   
    if ((*root) == nullptr || (*root)->tree_deleted) return;
    Push_Down(*root);     
    if (boxpoint.vertex_max[0] + EPSS < (*root)->node_range_x[0] || boxpoint.vertex_min[0] - EPSS > (*root)->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] + EPSS < (*root)->node_range_y[0] || boxpoint.vertex_min[1] - EPSS > (*root)->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] + EPSS < (*root)->node_range_z[0] || boxpoint.vertex_min[2] - EPSS > (*root)->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] - EPSS < (*root)->node_range_x[0] && boxpoint.vertex_max[0]+EPSS > (*root)->node_range_x[1] && boxpoint.vertex_min[1]-EPSS < (*root)->node_range_y[0] && boxpoint.vertex_max[1]+EPSS > (*root)->node_range_y[1] && boxpoint.vertex_min[2]-EPSS < (*root)->node_range_z[0] && boxpoint.vertex_max[2]+EPSS > (*root)->node_range_z[1]){
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
    if (boxpoint.vertex_min[0]-EPSS < (*root)->point.x && boxpoint.vertex_max[0]+EPSS > (*root)->point.x && boxpoint.vertex_min[1]-EPSS < (*root)->point.y && boxpoint.vertex_max[1]+EPSS > (*root)->point.y && boxpoint.vertex_min[2]-EPSS < (*root)->point.z && boxpoint.vertex_max[2]+EPSS > (*root)->point.z){
        (*root)->point_deleted = true;
        if (is_downsample) (*root)->point_downsample_deleted = true;       
    }
    Operation_Logger_Type delete_box_log;
    struct timespec Timeout;    
    if (is_downsample) delete_box_log.op = DOWNSAMPLE_DELETE;
        else delete_box_log.op = DELETE_BOX;
    delete_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
        Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        Delete_by_range(&((*root)->left_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){
        Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        Delete_by_range(&((*root)->right_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }    
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    return;
}

void KD_TREE::Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild){   
    if ((*root) == nullptr || (*root)->tree_deleted) return;
    Push_Down(*root);
    if (same_point((*root)->point, point) && !(*root)->point_deleted) {          
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize) (*root)->tree_deleted = true;    
        return;
    }
    Operation_Logger_Type delete_log;
    struct timespec Timeout;    
    delete_log.op = DELETE_POINT;
    delete_log.point = point;     
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z)){           
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){          
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);         
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->left_son_ptr, point,false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    } else {       
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);         
        } else {
            pthread_mutex_lock(&working_flag_mutex); 
            Delete_by_point(&(*root)->right_son_ptr, point, false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }        
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    return;
}

void KD_TREE::Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild){
    if ((*root) == nullptr) return;
    Push_Down(*root);       
    if (boxpoint.vertex_max[0] + EPSS < (*root)->node_range_x[0] || boxpoint.vertex_min[0] - EPSS > (*root)->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] + EPSS < (*root)->node_range_y[0] || boxpoint.vertex_min[1] - EPSS > (*root)->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] + EPSS < (*root)->node_range_z[0] || boxpoint.vertex_min[2] - EPSS > (*root)->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] - EPSS < (*root)->node_range_x[0] && boxpoint.vertex_max[0]+EPSS > (*root)->node_range_x[1] && boxpoint.vertex_min[1]-EPSS < (*root)->node_range_y[0] && boxpoint.vertex_max[1]+EPSS > (*root)->node_range_y[1] && boxpoint.vertex_min[2]-EPSS < (*root)->node_range_z[0] && boxpoint.vertex_max[2]+EPSS > (*root)->node_range_z[1]){
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        // (*root)->invalid_point_num = 0; 
        return;
    }
    if (boxpoint.vertex_min[0]-EPSS < (*root)->point.x && boxpoint.vertex_max[0]+EPSS > (*root)->point.x && boxpoint.vertex_min[1]-EPSS < (*root)->point.y && boxpoint.vertex_max[1]+EPSS > (*root)->point.y && boxpoint.vertex_min[2]-EPSS < (*root)->point.z && boxpoint.vertex_max[2]+EPSS > (*root)->point.z){
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }
    Operation_Logger_Type add_box_log;
    struct timespec Timeout;    
    add_box_log.op = ADD_BOX;
    add_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
        Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->left_son_ptr), boxpoint, false);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }        
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){
        Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->right_son_ptr), boxpoint, false);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    return;
}

void KD_TREE::Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild){     
    if (*root == nullptr){
        *root = new KD_TREE_NODE;
        InitTreeNode(*root);
        (*root)->point = point;
        (*root)->division_axis = rand() % 3;
        Update(*root);
        return;
    }           
    Operation_Logger_Type add_log;
    struct timespec Timeout;    
    add_log.op = ADD_POINT;
    add_log.point = point; 
    Push_Down(*root);
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z)){
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){          
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild);
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->left_son_ptr, point, false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);            
        }
    } else {  
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
            Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild);
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->right_son_ptr, point, false);       
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex); 
        }
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
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
    int cur_search_counter;
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    if (q.size()< k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist){
        if (dist_left_node <= dist_right_node) {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                Search(root->left_son_ptr, k_nearest, point,q);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point,q);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist) {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                    Search(root->right_son_ptr, k_nearest, point,q);                       
                } else {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);                    
                    Search(root->right_son_ptr, k_nearest, point,q);  
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }                
            }
        } else {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                Search(root->right_son_ptr, k_nearest, point,q);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);                   
                Search(root->right_son_ptr, k_nearest, point,q);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist) {            
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                    Search(root->left_son_ptr, k_nearest, point,q);                       
                } else {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);  
                    Search(root->left_son_ptr, k_nearest, point,q);  
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
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
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);  
                Search(root->left_son_ptr, k_nearest, point,q);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
        if (dist_right_node < q.top().dist) {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                Search(root->right_son_ptr, k_nearest, point,q);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);  
                Search(root->right_son_ptr, k_nearest, point,q);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    }  
    return;
}

void KD_TREE::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector & Storage){
    if (root == nullptr) return;
    Push_Down(root);       
    if (boxpoint.vertex_max[0] + EPSS < root->node_range_x[0] || boxpoint.vertex_min[0] - EPSS > root->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] + EPSS < root->node_range_y[0] || boxpoint.vertex_min[1] - EPSS > root->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] + EPSS < root->node_range_z[0] || boxpoint.vertex_min[2] - EPSS > root->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] - EPSS < root->node_range_x[0] && boxpoint.vertex_max[0]+EPSS > root->node_range_x[1] && boxpoint.vertex_min[1]-EPSS < root->node_range_y[0] && boxpoint.vertex_max[1]+EPSS > root->node_range_y[1] && boxpoint.vertex_min[2]-EPSS < root->node_range_z[0] && boxpoint.vertex_max[2]+EPSS > root->node_range_z[1]){
        flatten(root, Storage);
        return;
    }
    if (boxpoint.vertex_min[0]-EPSS < root->point.x && boxpoint.vertex_max[0]+EPSS > root->point.x && boxpoint.vertex_min[1]-EPSS < root->point.y && boxpoint.vertex_max[1]+EPSS > root->point.y && boxpoint.vertex_min[2]-EPSS < root->point.z && boxpoint.vertex_max[2]+EPSS > root->point.z){
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
    balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize-1);  
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
    Operation_Logger_Type operation;
    operation.op = PUSH_DOWN;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;
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
            pthread_mutex_lock(&working_flag_mutex);
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_left = false;
            pthread_mutex_unlock(&working_flag_mutex);            
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
            pthread_mutex_lock(&working_flag_mutex);
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }            
            root->need_push_down_to_right = false;
            pthread_mutex_unlock(&working_flag_mutex);
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
    if (root == Root_Node && root->TreeSize > 3){
        KD_TREE_NODE * son_ptr = root->left_son_ptr;
        if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize-1);
        root->alpha_del = float(root->invalid_point_num)/ root->TreeSize;
        root->alpha_bal = (tmp_bal>=0.5-EPSS)?tmp_bal:1-tmp_bal;
    }
    return;
}

void KD_TREE::flatten(KD_TREE_NODE * root, PointVector &Storage){
    if (root == nullptr || root->tree_deleted) return;
    Push_Down(root);
    if (!root->point_deleted) {
        Storage.push_back(root->point);
    }
    flatten(root->left_son_ptr, Storage);
    flatten(root->right_son_ptr, Storage);
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
        if ((*root)->point_deleted && !(*root)->point_downsample_deleted) {
            Points_deleted.push_back((*root)->point);
        }       
        break;
    case MULTI_THREAD_REC:
        pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);    
        if ((*root)->point_deleted  && !(*root)->point_downsample_deleted) {
            Multithread_Points_deleted.push_back((*root)->point);
        }
        pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);     
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
    return (fabs(a.x-b.x) < EPSS && fabs(a.y-b.y) < EPSS && fabs(a.z-b.z) < EPSS );
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

void KD_TREE::print_tree(int index, FILE *fp, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max){
    pthread_mutex_lock(&working_flag_mutex);
    print_treenode(Root_Node, index, fp, x_min,x_max,y_min,y_max,z_min,z_max);
    pthread_mutex_unlock(&working_flag_mutex);       
}

void KD_TREE::print_treenode(KD_TREE_NODE * root, int index, FILE *fp, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max){
    if (root == nullptr) return;
    Push_Down(root);
    fprintf(fp,"%d,%0.3f,%0.3f,%0.3f",index,root->point.x,root->point.y,root->point.z);
    fprintf(fp,",%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",x_min,x_max,y_min,y_max,z_min,z_max);
    switch (root->division_axis)
    {
    case 0:
        print_treenode(root->left_son_ptr, index, fp, x_min,root->point.x,y_min,y_max,z_min,z_max);
        print_treenode(root->right_son_ptr,index, fp, root->point.x,x_max,y_min,y_max,z_min,z_max);  
        break;
    case 1:
        print_treenode(root->left_son_ptr, index, fp, x_min,x_max,y_min,root->point.y,z_min,z_max);
        print_treenode(root->right_son_ptr,index, fp, x_min,x_max,root->point.y,y_max,z_min,z_max);   
        break;
    case 2:
        print_treenode(root->left_son_ptr, index, fp, x_min,x_max,y_min,y_max,z_min,root->point.z);
        print_treenode(root->right_son_ptr,index, fp, x_min,x_max,y_min,y_max,root->point.z,z_max);   
        break;             
    default:
        break;
    }
    return;    
}