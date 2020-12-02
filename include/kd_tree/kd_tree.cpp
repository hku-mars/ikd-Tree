#include <kd_tree.h>

/*
Description: K-D Tree improved for a better data structure performance. 
Author: Yixi CAI
email: yixicai@connect.hku.hk
*/

KD_TREE::~KD_TREE()
{
    // Release all pointer!
}

void KD_TREE::set_delete_criterion_param(float delete_param){
    delete_criterion_param = delete_param;
}

void KD_TREE::set_balance_criterion_param(float balance_param){
    balance_criterion_param = balance_param;
}

void KD_TREE::Build(vector<PointType> point_cloud){
    if (Root_Node != nullptr){
        // Release the existing kd-tree pointers
    }
    PCL_Storage = point_cloud;
    BuildTree(Root_Node, 0, PCL_Storage.size());
}

void KD_TREE::Nearest_Search(PointType point, int k_nearest, PointType * Nearest_Points){

}

void KD_TREE::Add_Points(vector<PointType> PointToAdd, int PointNum){
    for (int i=0; i<PointToAdd.size();i++){
        Add(Root_Node, PointToAdd[i]);
    }
    return;
}

void KD_TREE::Delete_Points(vector<PointType> PointToDel, int PointNum){
    for (int i=0;i!=PointToDel.size();i++){
        Delete_by_point(Root_Node, PointToDel[i]);
    }
    return;
}

void KD_TREE::Delete_Point_Boxes(float box_x_range[][2], float box_y_range[][2], float box_z_range[][2], int Box_Number){
    for (int i=0;i<Box_Number;i++){
        Delete_by_range(Root_Node , box_x_range[i], box_y_range[i], box_z_range[i]);
    }
    return;
}

void KD_TREE::BuildTree(KD_TREE_NODE * root, int l, int r){
    if (l>r) return;
    root = new KD_TREE_NODE;
    int mid = (l+r)/2;
    root->point = PCL_Storage[mid];
    // Find the best division Axis
    int i;
    float average[3] = {0,0,0};
    float covariance[3] = {0,0,0};
    for (i=l;i<=r;i++){
        average[0] = average[0] + PCL_Storage[i].x;
        average[1] = average[1] + PCL_Storage[i].y;
        average[2] = average[2] + PCL_Storage[i].z;
    }
    for (i=l;i<=r;i++){
        covariance[0] = (PCL_Storage[i].x - average[0]) * (PCL_Storage[i].x - average[0]);
        covariance[1] = (PCL_Storage[i].y - average[1]) * (PCL_Storage[i].y - average[1]);  
        covariance[2] = (PCL_Storage[i].z - average[2]) * (PCL_Storage[i].z - average[2]);              
    }
    // Select the axis with maximal covariance
    // Divide by the selection axis
    Pull_Up(root);
    // Check if rebuild is needed
}

void KD_TREE::Rebuild(KD_TREE_NODE * root){

}

void KD_TREE::Delete_by_range(KD_TREE_NODE * root, float x_range[], float y_range[], float z_range[]){
    // Deal with the condition that: push down label ON and new range delete command arrives -> compare the range and push down?
}

void KD_TREE::Delete_by_point(KD_TREE_NODE * root, PointType point){
    Push_Down(root);
    Pull_Up(root);
    // Check and rebuild    
}

void KD_TREE::Add(KD_TREE_NODE * root, PointType point){
    Push_Down(root);
    Pull_Up(root);
    // Check and rebuild    
}

void KD_TREE::Search(KD_TREE_NODE * root, int k_nearest){
    Push_Down(root);
    Pull_Up(root);
    // Check and rebuild
}

bool KD_TREE::Criterion_Check(KD_TREE_NODE * root){
    if (root->TreeSize == 1){
        return false;
    }
    float balance_evaluation = 0.0;
    float delete_evaluation = 0.0;
    KD_TREE_NODE * son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
    balance_evaluation = root->invalid_point_num/ root->TreeSize;
    delete_evaluation = son_ptr->TreeSize / root->TreeSize;
    if (delete_evaluation > delete_criterion_param){
        return true;
    }
    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1-balance_criterion_param){
        return true;
    } 
    return false;
}

void KD_TREE::Push_Down(KD_TREE_NODE * root){
    if (!root->delete_range_need_push_down){
        return;
    }
    KD_TREE_NODE * left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE * right_son_ptr = root->right_son_ptr;
    if (left_son_ptr != nullptr){
        left_son_ptr->delete_range_need_push_down = true;
        switch (root->division_axis)
        {
        case 0:
            left_son_ptr->delete_range_x[0] = root->delete_range_x[0];
            left_son_ptr->delete_range_x[1] = root->point.x;
            left_son_ptr->delete_range_y[0] = root->delete_range_y[0];
            left_son_ptr->delete_range_y[1] = root->delete_range_y[1];   
            left_son_ptr->delete_range_z[0] = root->delete_range_z[0];
            left_son_ptr->delete_range_z[1] = root->delete_range_z[1];                       
            break;
        case 1:
            left_son_ptr->delete_range_x[0] = root->delete_range_x[0];
            left_son_ptr->delete_range_x[1] = root->delete_range_x[1];
            left_son_ptr->delete_range_y[0] = root->delete_range_y[0];
            left_son_ptr->delete_range_y[1] = root->point.y;   
            left_son_ptr->delete_range_z[0] = root->delete_range_z[0];
            left_son_ptr->delete_range_z[1] = root->delete_range_z[1];          
            break;
        case 2:
            left_son_ptr->delete_range_x[0] = root->delete_range_x[0];
            left_son_ptr->delete_range_x[1] = root->delete_range_x[1];
            left_son_ptr->delete_range_y[0] = root->delete_range_y[0];
            left_son_ptr->delete_range_y[1] = root->delete_range_y[1];   
            left_son_ptr->delete_range_z[0] = root->delete_range_z[0];
            left_son_ptr->delete_range_z[1] = root->point.z;          
            break;
        default:
            left_son_ptr->delete_range_x[0] = root->delete_range_x[0];
            left_son_ptr->delete_range_x[1] = root->delete_range_x[1];
            left_son_ptr->delete_range_y[0] = root->delete_range_y[0];
            left_son_ptr->delete_range_y[1] = root->delete_range_y[1];   
            left_son_ptr->delete_range_z[0] = root->delete_range_z[0];
            left_son_ptr->delete_range_z[1] = root->delete_range_z[1];           
            break;
        }
    }
    if (right_son_ptr != nullptr){
        right_son_ptr->delete_range_need_push_down = true;
        switch (root->division_axis)
        {
        case 0:
            right_son_ptr->delete_range_x[0] = root->point.x;
            right_son_ptr->delete_range_x[1] = root->delete_range_x[1];
            right_son_ptr->delete_range_y[0] = root->delete_range_y[0];
            right_son_ptr->delete_range_y[1] = root->delete_range_y[1];   
            right_son_ptr->delete_range_z[0] = root->delete_range_z[0];
            right_son_ptr->delete_range_z[1] = root->delete_range_z[1];                       
            break;
        case 1:
            right_son_ptr->delete_range_x[0] = root->delete_range_x[0];
            right_son_ptr->delete_range_x[1] = root->delete_range_x[1];
            right_son_ptr->delete_range_y[0] = root->point.y;  
            right_son_ptr->delete_range_y[1] = root->delete_range_y[1]; 
            right_son_ptr->delete_range_z[0] = root->delete_range_z[0];
            right_son_ptr->delete_range_z[1] = root->delete_range_z[1];          
            break;
        case 2:
            right_son_ptr->delete_range_x[0] = root->delete_range_x[0];
            right_son_ptr->delete_range_x[1] = root->delete_range_x[1];
            right_son_ptr->delete_range_y[0] = root->delete_range_y[0];
            right_son_ptr->delete_range_y[1] = root->delete_range_y[1];   
            right_son_ptr->delete_range_z[0] = root->point.z;  
            right_son_ptr->delete_range_z[1] = root->delete_range_z[1];        
            break;
        default:
            right_son_ptr->delete_range_x[0] = root->delete_range_x[0];
            right_son_ptr->delete_range_x[1] = root->delete_range_x[1];
            right_son_ptr->delete_range_y[0] = root->delete_range_y[0];
            right_son_ptr->delete_range_y[1] = root->delete_range_y[1];   
            right_son_ptr->delete_range_z[0] = root->delete_range_z[0];
            right_son_ptr->delete_range_z[1] = root->delete_range_z[1];           
            break;
        }        
    }
    root->delete_range_need_push_down = false;
}

void KD_TREE::Pull_Up(KD_TREE_NODE* root){
    KD_TREE_NODE * left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE * right_son_ptr = root->right_son_ptr;
    // Update Tree Size
    if (left_son_ptr != nullptr && right_son_ptr != nullptr){
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + root->point_deleted? 0:1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + root->point_deleted? 1:0;
        root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;
        root->node_range_x[0] = min(min(left_son_ptr->node_range_x[0],right_son_ptr->node_range_x[0]),root->point.x);
        root->node_range_x[1] = max(max(left_son_ptr->node_range_x[1],right_son_ptr->node_range_x[1]),root->point.x);
        root->node_range_y[0] = min(min(left_son_ptr->node_range_y[0],right_son_ptr->node_range_y[0]),root->point.y);
        root->node_range_y[1] = max(max(left_son_ptr->node_range_y[1],right_son_ptr->node_range_y[1]),root->point.y);        
        root->node_range_z[0] = min(min(left_son_ptr->node_range_z[0],right_son_ptr->node_range_z[0]),root->point.z);
        root->node_range_z[1] = max(max(left_son_ptr->node_range_z[1],right_son_ptr->node_range_z[1]),root->point.z);         
    } else if (left_son_ptr != nullptr){
        root->TreeSize = left_son_ptr->TreeSize + root->point_deleted? 0:1;;
        root->invalid_point_num = left_son_ptr->invalid_point_num + root->point_deleted? 1:0;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        root->node_range_x[0] = min(left_son_ptr->node_range_x[0],root->point.x);
        root->node_range_x[1] = max(left_son_ptr->node_range_x[1],root->point.x);
        root->node_range_y[0] = min(left_son_ptr->node_range_y[0],root->point.y);
        root->node_range_y[1] = max(left_son_ptr->node_range_y[1],root->point.y); 
        root->node_range_z[0] = min(left_son_ptr->node_range_z[0],root->point.z);
        root->node_range_z[1] = max(left_son_ptr->node_range_z[1],root->point.z);               
    } else if (right_son_ptr != nullptr){
        root->TreeSize = right_son_ptr->TreeSize + root->point_deleted? 0:1;
        root->invalid_point_num = right_son_ptr->invalid_point_num + root->point_deleted? 1:0;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;        
        root->node_range_x[0] = min(right_son_ptr->node_range_x[0],root->point.x);
        root->node_range_x[1] = max(right_son_ptr->node_range_x[1],root->point.x);
        root->node_range_y[0] = min(right_son_ptr->node_range_y[0],root->point.y);
        root->node_range_y[1] = max(right_son_ptr->node_range_y[1],root->point.y); 
        root->node_range_z[0] = min(right_son_ptr->node_range_z[0],root->point.z);
        root->node_range_z[1] = max(right_son_ptr->node_range_z[1],root->point.z);        
    } else {
        root->TreeSize = root->point_deleted? 0:1;
        root->invalid_point_num = root->point_deleted? 1:0;
        root->tree_deleted = root->point_deleted;
        root->node_range_x[0] = root->point.x;
        root->node_range_x[1] = root->point.x;        
        root->node_range_y[0] = root->point.y;
        root->node_range_y[1] = root->point.y; 
        root->node_range_z[0] = root->point.z;
        root->node_range_z[1] = root->point.z;                 
    }
    return;
}
