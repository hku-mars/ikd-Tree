#include <pcl/kdtree/kdtree_flann.h>
#include <kd_tree/kd_tree.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <random>
#include <algorithm>
#include <chrono>

#define X_MAX 3
#define X_MIN -3
#define Y_MAX 3
#define Y_MIN 0
#define Z_MAX 2
#define Z_MIN 0

#define Point_Num 50000
#define New_Point_Num 250
#define Delete_Point_Num 200
#define Nearest_Num 5
#define Test_Time 100
#define Search_Time 400
#define Box_Length 1
#define Box_Num 1

vector<PointType> point_cloud;
vector<PointType> cloud_increment;
vector<PointType> cloud_decrement;
vector<PointType> search_result;
vector<PointType> raw_cmp_result;

KD_TREE scapegoat_kd_tree(0.5,0.7,0.1,10);

float rand_float(float x_min, float x_max){
    float rand_ratio = rand()/(float)RAND_MAX;
    return (x_min + rand_ratio * (x_max - x_min));
}

void generate_initial_point_cloud(int num){
    vector<PointType> ().swap(point_cloud);
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
    vector<PointType> ().swap(cloud_increment);
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
            if (tmp.x +EPS < boxpoint.vertex_min[0] || tmp.x - EPS > boxpoint.vertex_max[0] || tmp.y + EPS < boxpoint.vertex_min[1] || tmp.y - EPS > boxpoint.vertex_max[1] || tmp.z + EPS < boxpoint.vertex_min[2] || tmp.z - EPS > boxpoint.vertex_max[2]){
                point_cloud.insert(point_cloud.begin(),tmp);
            }
            counter += 1;
        }   
    //printf("x:(%0.3f %0.3f) y:(%0.3f %0.3f) z:(%0.3f %0.3f)\n",x_r[k][0],x_r[k][1],y_r[k][0],y_r[k][1],z_r[k][0],z_r[k][1]); 
    }

}

PointType generate_target_point(){
    PointType point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    return point;
}

void generate_decrement_point_cloud(int num){
    vector<PointType> ().swap(cloud_decrement);
    auto rng = default_random_engine();
    shuffle(point_cloud.begin(), point_cloud.end(), rng);    
    for (int i=0;i<num;i++){
        cloud_decrement.push_back(point_cloud[point_cloud.size()-1]);
        point_cloud.pop_back();
    }
    return;
}

void print_point_vec(vector<PointType> vec){
    printf("Size is %d\n", int(vec.size()));
    for (int i=0;i<vec.size();i++){
        printf("(%0.3f, %0.3f, %0.3f)\n",vec[i].x,vec[i].y,vec[i].z);
    }
    return;
}


void raw_cmp(PointType target, int k_nearest){
    vector<PointType_CMP> points;
    vector<PointType> ().swap(raw_cmp_result);
    float dist;
    for (int i=0;i<point_cloud.size();i++){
        dist = (point_cloud[i].x-target.x)*(point_cloud[i].x-target.x) + (point_cloud[i].y-target.y)*(point_cloud[i].y-target.y) + (point_cloud[i].z-target.z)*(point_cloud[i].z-target.z);
        PointType_CMP tmp{point_cloud[i],dist};
        points.push_back(tmp);
    }
    sort(points.begin(),points.end());
    for (int i=0;i<k_nearest;i++){
        raw_cmp_result.push_back(points[i].point);
    }
    return;
}

bool cmp_point_vec(vector<PointType> a, vector<PointType> b){
    if (a.size() != b.size()) return false;
    for (int i =0;i<a.size();i++){
        if (fabs(a[i].x-b[i].x)>EPS || fabs(a[i].y-b[i].y)>EPS || fabs(a[i].y-b[i].y)>EPS) return false;
    }
    return true;
}

int main(int argc, char** argv){
    srand((unsigned) time(NULL));
    printf("Testing ...\n");
    int counter = 0;
    bool flag = true;
    vector<BoxPointType> Delete_Boxes;
    vector<PointType> DeletePoints;
    float max_total_time = 0.0;
    float box_delete_time = 0.0;
    float add_time = 0.0;
    float delete_time = 0.0;
    float search_time = 0.0;
    int max_point_num = 0;
    int point_num_start = 0;
    int rebuild_counter = 0, add_rebuild_record = 0;
    PointType target; 
    // Initialize k-d tree
    FILE *fp_log;
    fp_log = freopen("kd_tree_test_log.txt","w",stdout);
    fprintf(fp_log,"Add, Delete Points, Delete Boxes, Search, Total\n");
    fclose(stdout);
    generate_initial_point_cloud(Point_Num);
    scapegoat_kd_tree.Build(point_cloud);    
    while (flag && counter < Test_Time){
        printf("Test %d:\n",counter+1);
        point_num_start = scapegoat_kd_tree.Root_Node->TreeSize;
        // Incremental Operation
        generate_increment_point_cloud(New_Point_Num);
        auto t1 = chrono::high_resolution_clock::now();
        scapegoat_kd_tree.Add_Points(cloud_increment);
        auto t2 = chrono::high_resolution_clock::now();
        auto add_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();
        auto total_duration = add_duration;
        add_rebuild_record = scapegoat_kd_tree.rebuild_counter;
        printf("Add point time cost is %0.3f ms\n",float(add_duration)/1e3);
        // Decremental Operation
        generate_decrement_point_cloud(Delete_Point_Num);     
        t1 = chrono::high_resolution_clock::now();
        scapegoat_kd_tree.Delete_Points(cloud_decrement);
        t2 = chrono::high_resolution_clock::now();
        auto delete_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();
        total_duration += delete_duration;
        printf("Delete point time cost is %0.3f ms\n",float(delete_duration)/1e3);      
        auto box_delete_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();        
        // Box Decremental Operation
        if ((counter+1) % 20  == 0 ){
            generate_box_decrement(Delete_Boxes, Box_Length, Box_Num);
            t1 = chrono::high_resolution_clock::now();
            scapegoat_kd_tree.Delete_Point_Boxes(Delete_Boxes);
            t2 = chrono::high_resolution_clock::now();
            box_delete_duration = chrono::duration_cast<chrono::microseconds>(t2-t1).count();
            total_duration += box_delete_duration;
            printf("Delete box points time cost is %0.3f ms\n",float(box_delete_duration)/1e3); 
        }
        // Search Operation
        auto search_duration = chrono::duration_cast<chrono::microseconds>(t2-t2).count();            
        for (int k=0;k<Search_Time;k++){
            vector<PointType> ().swap(search_result);             
            target = generate_target_point();    
            t1 = chrono::high_resolution_clock::now();
            scapegoat_kd_tree.Nearest_Search(target, Nearest_Num, search_result);
            t2 = chrono::high_resolution_clock::now();
            search_duration += chrono::duration_cast<chrono::microseconds>(t2-t1).count();
        }
        printf("Search nearest point time cost is %0.3f ms\n",float(search_duration)/1e3);
        total_duration += search_duration;
        printf("Total time is %0.3f ms\n",total_duration/1e3);
        if (float(total_duration) > max_total_time){
            max_total_time = float(total_duration);
            box_delete_time = box_delete_duration;
            add_time = add_duration;
            delete_time = delete_duration;
            search_time = search_duration;            
            max_point_num = point_num_start;
            rebuild_counter = add_rebuild_record;
        }
        max_total_time = max(max_total_time, float(total_duration));
        raw_cmp(target, Nearest_Num);    
        flag = cmp_point_vec(search_result, raw_cmp_result);      
        counter += 1;    
        fp_log = freopen("kd_tree_test_log.txt","w",stdout);
        fprintf(fp_log,"%f,%f,%f,%f,%f\n",add_duration/1e3,delete_duration/1e3,box_delete_duration/1e3,search_duration/1e3,total_duration/1e3);
        fclose(stdout);        
    }
    // printf("Point Cloud Points:\n");
    // printf("Target Point is : (%0.3f, %0.3f, %0.3f)\n", target.x, target.y, target.z);
    FILE *fp;
    if (!flag & (scapegoat_kd_tree.Root_Node==nullptr || scapegoat_kd_tree.Root_Node->TreeSize >=5)){
        printf("Find Dataset for debug!\n");
        fp = freopen("Data_for_fault.txt","w",stdout);
        for (int i=0;i<point_cloud.size();i++){
            fprintf(fp,"%0.6f,%0.6f,%0.6f\n",point_cloud[i].x,point_cloud[i].y,point_cloud[i].z);
        }
        printf("Raw cmp:\n");
        print_point_vec(raw_cmp_result);
        printf("k d tree\n");
        print_point_vec(search_result);
        printf("Points in kd_tree\n");
        vector<PointType> ().swap(scapegoat_kd_tree.PCL_Storage);
        if (scapegoat_kd_tree.Root_Node != nullptr) scapegoat_kd_tree.traverse_for_rebuild(scapegoat_kd_tree.Root_Node);
        print_point_vec(scapegoat_kd_tree.PCL_Storage);   
        fclose(stdout);        
    } else {
        printf("Finished %d times test\n",counter);
        printf("Max Total Time is: %0.3fms\n",max_total_time/1e3);
        printf("Add Time is: %0.3fms with rebuild counter %d\n",add_time/1e3,rebuild_counter);        
        printf("Delete Time is: %0.3fms\n",delete_time/1e3);
        printf("Box delete Time is: %0.3fms\n",box_delete_time/1e3);     
        printf("Search Time is: %0.3fms\n",search_time/1e3);           
        printf("Corresponding point number is: %d\n",max_point_num);
    }
    return 0;
}
