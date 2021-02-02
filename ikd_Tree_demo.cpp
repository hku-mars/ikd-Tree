#include "ikd_Tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>



#define X_MAX 5.0
#define X_MIN -5.0
#define Y_MAX 5.0
#define Y_MIN -5.0
#define Z_MAX 5.0
#define Z_MIN -5.0

#define Point_Num 4000
#define New_Point_Num 4000
#define Delete_Point_Num 0
#define Nearest_Num 5
#define Test_Time 5000
#define Search_Time 200
#define Box_Length 1.5
#define Box_Num 4
#define Delete_Box_Switch false
#define Add_Box_Switch false

PointVector point_cloud;
PointVector cloud_increment;
PointVector cloud_decrement;
PointVector cloud_deleted;
PointVector search_result;
PointVector raw_cmp_result;
PointVector DeletePoints;
PointVector removed_points;

KD_TREE scapegoat_kd_tree(0.3,0.6,0.2);

float rand_float(float x_min, float x_max){
    float rand_ratio = rand()/(float)RAND_MAX;
    return (x_min + rand_ratio * (x_max - x_min));
}

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
                // printf("        Decremental Push back: (%0.3f, %0.3f, %0.3f)\n",tmp.x,tmp.y,tmp.z);                              
            }
            counter += 1;
        }   
        printf("Deleted box: x:(%0.3f %0.3f) y:(%0.3f %0.3f) z:(%0.3f %0.3f)\n",boxpoint.vertex_min[0],boxpoint.vertex_max[0],boxpoint.vertex_min[1],boxpoint.vertex_max[1], boxpoint.vertex_min[2],boxpoint.vertex_max[2]); 
    }
}

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
                // printf("        Incremental Push back: (%0.3f, %0.3f, %0.3f)\n",tmp.x,tmp.y,tmp.z);                    
                point_cloud.push_back(tmp);
            }
            counter += 1;
        }
        printf("Add boxes: x:(%0.3f %0.3f) y:(%0.3f %0.3f) z:(%0.3f %0.3f)\n",boxpoint.vertex_min[0],boxpoint.vertex_max[0],boxpoint.vertex_min[1],boxpoint.vertex_max[1], boxpoint.vertex_min[2],boxpoint.vertex_max[2]); 
    }
}

PointType generate_target_point(){
    PointType point;
    point.x = rand_float(X_MIN, X_MAX);;
    point.y = rand_float(Y_MIN, Y_MAX);
    point.z = rand_float(Z_MIN, Z_MAX);
    return point;
}

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

int main(int argc, char** argv){
    srand((unsigned) time(NULL));
    printf("Testing ...\n");
    int counter = 0;
    bool flag = true;
    vector<BoxPointType> Delete_Boxes;
    vector<BoxPointType> Add_Boxes;
    vector<float> PointDist;
    float max_total_time = 0.0;
    float box_delete_time = 0.0;
    float box_add_time = 0.0;
    float add_time = 0.0;
    float delete_time = 0.0;
    float search_time = 0.0;
    float ori_build_time = 0.0;
    float ori_search_time = 0.0;
    int max_point_num = 0;
    int point_num_start = 0;
    int add_rebuild_record = 0, add_tmp_rebuild_counter = 0;
    int delete_rebuild_record = 0, delete_tmp_rebuild_counter = 0;    
    int delete_box_rebuild_record = 0, delete_box_tmp_rebuild_counter = 0;
    int wa_rec = 0;
    PointType target; 
    // Initialize k-d tree 
    generate_initial_point_cloud(Point_Num);
    auto t1 = chrono::high_resolution_clock::now();
    scapegoat_kd_tree.Build(point_cloud);    
    auto t2 = chrono::high_resolution_clock::now();    
    auto build_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();
    int counter_div = 0;
    while (counter < Test_Time){
        counter_div = int(counter/100 + EPSS);
        generate_initial_point_cloud(Point_Num * (counter_div + 1));
        scapegoat_kd_tree.Build(point_cloud);            
        printf("Test %d:\n",counter+1);      
        // Incremental Operation
        // if ((counter+1) % 100  == 0){
        //     printf("Large scale add\n");
        //     generate_increment_point_cloud(New_Point_Num * 10);
        // } else {
        //     generate_increment_point_cloud(New_Point_Num);
        // }
        generate_increment_point_cloud(New_Point_Num);
        printf("Start add\n");
        // printf("New Points are\n");
        // print_point_vec(cloud_increment);
        t1 = chrono::high_resolution_clock::now();
        scapegoat_kd_tree.Add_Points(cloud_increment, false);
        t2 = chrono::high_resolution_clock::now();
        auto add_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();      
        auto total_duration = add_duration;
        // add_rebuild_record = scapegoat_kd_tree.rebuild_counter;
        printf("Add point time cost is %0.3f ms\n",float(add_duration)/1e3);
        // Decremental Operation
        generate_decrement_point_cloud(Delete_Point_Num);     
        printf("Start delete\n");        
        t1 = chrono::high_resolution_clock::now();
        scapegoat_kd_tree.Delete_Points(cloud_decrement);
        t2 = chrono::high_resolution_clock::now();
        auto delete_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();       
        total_duration += delete_duration;
        printf("Delete point time cost is %0.3f ms\n",float(delete_duration)/1e3);      
        auto box_delete_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();
        // delete_tmp_rebuild_counter = scapegoat_kd_tree.rebuild_counter;        
        // Box Decremental Operation
        if (Delete_Box_Switch && (counter+1) % 50  == 0 && (counter+1) % 100 != 0){ 
            printf("Wrong answer with counter %d\n", wa_rec);               
            generate_box_decrement(Delete_Boxes, Box_Length, Box_Num);
            t1 = chrono::high_resolution_clock::now();
            scapegoat_kd_tree.Delete_Point_Boxes(Delete_Boxes);
            t2 = chrono::high_resolution_clock::now();            
            box_delete_duration += chrono::duration_cast<chrono::microseconds>(t2-t1).count();
            printf("Delete box points time cost is %0.3f ms\n",float(box_delete_duration)/1e3); 
            // delete_box_tmp_rebuild_counter = scapegoat_kd_tree.rebuild_counter;
        }
        total_duration += box_delete_duration;  
        auto box_add_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();        
        if (Add_Box_Switch && (counter+1) % 100  == 0){ 
            printf("Wrong answer with counter %d\n", wa_rec);               
            generate_box_increment(Add_Boxes, Box_Length, Box_Num);
            t1 = chrono::high_resolution_clock::now();
            scapegoat_kd_tree.Add_Point_Boxes(Add_Boxes);
            t2 = chrono::high_resolution_clock::now();            
            box_add_duration += chrono::duration_cast<chrono::microseconds>(t2-t1).count();
            printf("Add box points time cost is %0.3f ms\n",float(box_add_duration)/1e3); 
            // delete_box_tmp_rebuild_counter = scapegoat_kd_tree.rebuild_counter;
        }
        total_duration += box_add_duration;               
        // Search Operation       
        auto search_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();
        auto ori_search_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();
        printf("Start Search\n");               
        for (int k=0;k<Search_Time;k++){
            PointVector ().swap(search_result);             
            target = generate_target_point();    
            t1 = chrono::high_resolution_clock::now();
            scapegoat_kd_tree.Nearest_Search(target, Nearest_Num, search_result, PointDist);
            t2 = chrono::high_resolution_clock::now();
            search_duration += chrono::duration_cast<chrono::microseconds>(t2-t1).count();
        }
        printf("Search nearest point time cost is %0.3f ms\n",float(search_duration)/1e3);
        printf("Original Search nearest point time cost is %0.3f ms\n",float(ori_search_duration)/1e3);
        total_duration += search_duration;
        printf("Total time is %0.3f ms\n\n",total_duration/1e3);
        if (float(total_duration) > max_total_time){
            max_total_time = float(total_duration);
            box_delete_time = box_delete_duration;
            box_add_time = box_add_duration;
            add_time = add_duration;
            delete_time = delete_duration;
            search_time = search_duration;            
            max_point_num = point_num_start;
            add_rebuild_record = add_tmp_rebuild_counter;
            delete_rebuild_record = delete_tmp_rebuild_counter;
            delete_box_rebuild_record = delete_box_tmp_rebuild_counter;
        }
        max_total_time = max(max_total_time, float(total_duration)); 
        counter += 1;    
        PointVector ().swap(removed_points);
        scapegoat_kd_tree.acquire_removed_points(removed_points);
        // print_point_vec(removed_points);
    }
    usleep(1e5);
    // printf("Point Cloud Points:\n");
    // printf("Target Point is : (%0.3f, %0.3f, %0.3f)\n", target.x, target.y, target.z);
    if (!flag & (scapegoat_kd_tree.Root_Node==nullptr || scapegoat_kd_tree.Root_Node->TreeSize >=5)){
        scapegoat_kd_tree.flatten(scapegoat_kd_tree.Root_Node, scapegoat_kd_tree.PCL_Storage);    
    } else {
        printf("Finished %d times test\n",counter);
        printf("Max Total Time is: %0.3fms\n",max_total_time/1e3);
        printf("Add Time is: %0.3fms\n",add_time/1e3);        
        printf("Delete Time is: %0.3fms\n",delete_time/1e3);
        printf("Box delete Time is: %0.3fms\n",box_delete_time/1e3);    
        printf("Box Add Time is: %0.3fms\n",box_add_time/1e3);          
        printf("Search Time is: %0.3fms\n",search_time/1e3);           
        printf("Corresponding point number is: %d\n",max_point_num);
        PointVector ().swap(scapegoat_kd_tree.PCL_Storage);    
    }
    printf("WA counter is %d\n",wa_rec);
    return 0;
}