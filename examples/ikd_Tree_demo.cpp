/*
    Description: An example for using ikd-Tree
    Author: Yixi Cai
    Email: yixicai@connect.hku.hk
*/

#include <ikd_Tree.h>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>

using PointType = ikdTree_PointType;
using PointVector = KD_TREE<PointType>::PointVector;

#define X_MAX 5.0
#define X_MIN -5.0
#define Y_MAX 5.0
#define Y_MIN -5.0
#define Z_MAX 5.0
#define Z_MIN -5.0

#define Point_Num 20000
#define New_Point_Num 200
#define Delete_Point_Num 100
#define Nearest_Num 5
#define Test_Time 1000
#define Search_Counter 200
#define Box_Length 1.5
#define Box_Num 4
#define Delete_Box_Switch true
#define Add_Box_Switch true

PointVector point_cloud;
PointVector cloud_increment;
PointVector cloud_decrement;
PointVector cloud_deleted;
PointVector search_result;
PointVector raw_cmp_result;
PointVector DeletePoints;
PointVector removed_points;

KD_TREE<ikdTree_PointType> ikd_Tree(0.3,0.6,0.2);

float rand_float(float x_min, float x_max){
    float rand_ratio = rand()/(float)RAND_MAX;
    return (x_min + rand_ratio * (x_max - x_min));
}


/*
   Generate the points to initialize an incremental k-d tree
*/

void generate_initial_point_cloud(int num){
    PointVector ().swap(point_cloud);
    PointType new_point;
    for (int i=0;i<num;i++){
        new_point.x = rand_float(X_MIN, X_MAX);
        new_point.y = rand_float(Y_MIN, Y_MAX);
        new_point.z = rand_float(Z_MIN, Z_MAX);
        point_cloud.push_back(new_point);
    }
    return;
}

/*
    Generate random new points for point-wise insertion to the incremental k-d tree
*/

void generate_increment_point_cloud(int num){
    PointVector ().swap(cloud_increment);
    PointType new_point;
    for (int i=0;i<num;i++){
        new_point.x = rand_float(X_MIN, X_MAX);
        new_point.y = rand_float(Y_MIN, Y_MAX);
        new_point.z = rand_float(Z_MIN, Z_MAX);
        point_cloud.push_back(new_point);
        cloud_increment.push_back(new_point);        
    }
    return;
}


/*
    Generate random points for point-wise delete on the incremental k-d tree
*/

void generate_decrement_point_cloud(int num){
    PointVector ().swap(cloud_decrement);
    auto rng = default_random_engine();
    shuffle(point_cloud.begin(), point_cloud.end(), rng);    
    for (int i=0;i<num;i++){
        cloud_decrement.push_back(point_cloud[point_cloud.size()-1]);
        point_cloud.pop_back();
    }
    return;
}

/*
    Generate random boxes for box-wise re-insertion on the incremental k-d tree
*/

void generate_box_increment(vector<BoxPointType> & Add_Boxes, float box_length, int box_num){
    vector<BoxPointType> ().swap(Add_Boxes);
    float d = box_length/2;
    float x_p, y_p, z_p;
    BoxPointType boxpoint;
    for (int k=0;k < box_num; k++){
        x_p = rand_float(X_MIN, X_MAX);
        y_p = rand_float(Y_MIN, Y_MAX);
        z_p = rand_float(Z_MIN, Z_MAX);        
        boxpoint.vertex_min[0] = x_p - d;
        boxpoint.vertex_max[0] = x_p + d;
        boxpoint.vertex_min[1] = y_p - d;
        boxpoint.vertex_max[1] = y_p + d;  
        boxpoint.vertex_min[2] = z_p - d;
        boxpoint.vertex_max[2] = z_p + d;
        Add_Boxes.push_back(boxpoint);
        int n = cloud_deleted.size();
        int counter = 0;
        while (counter < n){
            PointType tmp = cloud_deleted[cloud_deleted.size()-1];
            cloud_deleted.pop_back();
        
            if (tmp.x +EPSS < boxpoint.vertex_min[0] || tmp.x - EPSS > boxpoint.vertex_max[0] || tmp.y + EPSS < boxpoint.vertex_min[1] || tmp.y - EPSS > boxpoint.vertex_max[1] || tmp.z + EPSS < boxpoint.vertex_min[2] || tmp.z - EPSS > boxpoint.vertex_max[2]){
                cloud_deleted.insert(cloud_deleted.begin(),tmp);
            } else {            
                point_cloud.push_back(tmp);
            }
            counter += 1;
        }
    }
}

/*
    Generate random boxes for box-wise delete on the incremental k-d tree
*/

void generate_box_decrement(vector<BoxPointType> & Delete_Boxes, float box_length, int box_num){
    vector<BoxPointType> ().swap(Delete_Boxes);
    float d = box_length/2;
    float x_p, y_p, z_p;
    BoxPointType boxpoint;
    for (int k=0;k < box_num; k++){
        x_p = rand_float(X_MIN, X_MAX);
        y_p = rand_float(Y_MIN, Y_MAX);
        z_p = rand_float(Z_MIN, Z_MAX);        
        boxpoint.vertex_min[0] = x_p - d;
        boxpoint.vertex_max[0] = x_p + d;
        boxpoint.vertex_min[1] = y_p - d;
        boxpoint.vertex_max[1] = y_p + d;  
        boxpoint.vertex_min[2] = z_p - d;
        boxpoint.vertex_max[2] = z_p + d;
        Delete_Boxes.push_back(boxpoint);
        int n = point_cloud.size();
        int counter = 0;
        while (counter < n){
            PointType tmp = point_cloud[point_cloud.size()-1];
            point_cloud.pop_back();            
            if (tmp.x +EPSS < boxpoint.vertex_min[0] || tmp.x - EPSS > boxpoint.vertex_max[0] || tmp.y + EPSS < boxpoint.vertex_min[1] || tmp.y - EPSS > boxpoint.vertex_max[1] || tmp.z + EPSS < boxpoint.vertex_min[2] || tmp.z - EPSS > boxpoint.vertex_max[2]){
                point_cloud.insert(point_cloud.begin(),tmp);                  
            } else {
                cloud_deleted.push_back(tmp);
            }
            counter += 1;
        }   
    }
}


/*
    Generate target point for nearest search on the incremental k-d tree
*/

PointType generate_target_point(){
    PointType point;
    point.x = rand_float(X_MIN, X_MAX);;
    point.y = rand_float(Y_MIN, Y_MAX);
    point.z = rand_float(Z_MIN, Z_MAX);
    return point;
}

int main(int argc, char** argv){
    srand((unsigned) time(NULL));
    printf("Testing ...\n");
    int counter = 0;
    bool flag = true;
    vector<BoxPointType> Delete_Boxes;
    vector<BoxPointType> Add_Boxes;
    vector<float> PointDist;
    float average_total_time = 0.0;
    float box_delete_time = 0.0;
    float box_add_time = 0.0;
    float add_time = 0.0;
    float delete_time = 0.0;
    float search_time = 0.0;
    int box_delete_counter = 0;
    int box_add_counter = 0;
    PointType target; 
    // Initialize k-d tree
    generate_initial_point_cloud(Point_Num);
    auto t1 = chrono::high_resolution_clock::now();
    ikd_Tree.Build(point_cloud);    
    auto t2 = chrono::high_resolution_clock::now();    
    auto build_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();
    while (counter < Test_Time){           
        printf("Test %d:\n",counter+1);
        // Point-wise Insertion
        generate_increment_point_cloud(New_Point_Num);
        t1 = chrono::high_resolution_clock::now();
        ikd_Tree.Add_Points(cloud_increment, false);
        t2 = chrono::high_resolution_clock::now();
        auto add_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();      
        auto total_duration = add_duration;
        printf("Add point time cost is %0.3f ms\n",float(add_duration)/1e3);
        // Point-wise Delete
        generate_decrement_point_cloud(Delete_Point_Num);            
        t1 = chrono::high_resolution_clock::now();
        ikd_Tree.Delete_Points(cloud_decrement);
        t2 = chrono::high_resolution_clock::now();
        auto delete_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();       
        total_duration += delete_duration;
        printf("Delete point time cost is %0.3f ms\n",float(delete_duration)/1e3);      
        // Box-wise Delete
        auto box_delete_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();
        if (Delete_Box_Switch && (counter+1) % 500  == 0){
            printf("Waiting to generate 4 cuboids for box-wise delete test...\n");
            generate_box_decrement(Delete_Boxes, Box_Length, Box_Num);
            t1 = chrono::high_resolution_clock::now();
            ikd_Tree.Delete_Point_Boxes(Delete_Boxes);
            t2 = chrono::high_resolution_clock::now();            
            box_delete_counter ++;
            box_delete_duration += chrono::duration_cast<chrono::microseconds>(t2-t1).count();
            printf("Delete box points time cost is %0.3f ms\n",float(box_delete_duration)/1e3); 
        }
        total_duration += box_delete_duration;  
        // Box-wise Re-insertion 
        auto box_add_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();        
        if (Add_Box_Switch && (counter+1) % 100  == 0){ 
            generate_box_increment(Add_Boxes, Box_Length, Box_Num);
            t1 = chrono::high_resolution_clock::now();
            ikd_Tree.Add_Point_Boxes(Add_Boxes);
            t2 = chrono::high_resolution_clock::now();            
            box_add_counter ++;
            box_add_duration += chrono::duration_cast<chrono::microseconds>(t2-t1).count();
            printf("Add box points time cost is %0.3f ms\n",float(box_add_duration)/1e3); 
        }
        total_duration += box_add_duration;               
        // Nearest Search    
        auto search_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();
        for (int k=0;k<Search_Counter;k++){
            PointVector ().swap(search_result);             
            target = generate_target_point();    
            t1 = chrono::high_resolution_clock::now();
            ikd_Tree.Nearest_Search(target, Nearest_Num, search_result, PointDist);
            t2 = chrono::high_resolution_clock::now();
            search_duration += chrono::duration_cast<chrono::microseconds>(t2-t1).count();
        }
        printf("Search nearest point time cost is %0.3f ms\n",float(search_duration)/1e3);
        total_duration += search_duration;
        printf("Total time is %0.3f ms\n",total_duration/1e3);
        printf("Tree size is: %d\n\n", ikd_Tree.size());
        // If necessary, the removed points can be collected.
        PointVector ().swap(removed_points);
        ikd_Tree.acquire_removed_points(removed_points);
        // Calculate total running time
        average_total_time += float(total_duration)/1e3;
        box_delete_time += float(box_delete_duration)/1e3;
        box_add_time += float(box_add_duration)/1e3;
        add_time += float(add_duration)/1e3;
        delete_time += float(delete_duration)/1e3;
        search_time += float(search_duration)/1e3; 
        counter += 1;    
    }

    printf("Finished %d times test\n",counter);
    printf("Average Time:\n");
    printf("Total Time is: %0.3fms\n",average_total_time/1e3);
    printf("Point-wise Insertion (%d points): %0.3fms\n",New_Point_Num,add_time/counter);        
    printf("Point-wise Delete (%d points):    %0.3fms\n", Delete_Point_Num,delete_time/counter);
    printf("Box-wse Delete (%d boxes):        %0.3fms\n",Box_Num,box_delete_time/box_delete_counter);    
    printf("Box-wse Re-insertion (%d boxes):  %0.3fms\n",Box_Num,box_add_time/box_add_counter);          
    printf("Nearest Search (%d points):       %0.3fms\n", Search_Counter,search_time/counter);              
    return 0;
}