#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <fstream>
#include <sstream>
#include <pcl/features/normal_3d.h> 
#include <ctime>
#include <time.h>

void knn(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index, std::vector<int>& pointIdxNKNSearch){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    int K = 3;
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (cloud->points[index], K, pointIdxNKNSearch, pointNKNSquaredDistance);
}

void cal_2_norm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> cal_idx, Eigen::Vector3f& vec_1, Eigen::Vector3f& vec_2){
    Eigen::Vector3f c_0(cloud->points[cal_idx[1]].x-cloud->points[cal_idx[0]].x,
    cloud->points[cal_idx[1]].y-cloud->points[cal_idx[0]].y,
    cloud->points[cal_idx[1]].z-cloud->points[cal_idx[0]].z);

    Eigen::Vector3f c_1(cloud->points[cal_idx[2]].x-cloud->points[cal_idx[1]].x,
    cloud->points[cal_idx[2]].y-cloud->points[cal_idx[1]].y,
    cloud->points[cal_idx[2]].z-cloud->points[cal_idx[1]].z);

    vec_1 = c_0.cross(c_1);
    vec_1 /= vec_1.norm();
    vec_2 = c_1.cross(c_0);
    vec_2 /= vec_2.norm();
}

void cal_nearest_pt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f normal, Eigen::Vector3f pt, std::vector<float>& distance_down){
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);
    int step = 100;
    float move = 5.0;
    float interval = 2.0; //direction : 0,1,0 baecause in this house, vertical axis is y-axis.

    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    Eigen::Vector3f disturb_pt = pt + move*normal;

    for(int i=0;i<step;i++){
        pcl::PointXYZ point;
        point.x = disturb_pt(0)+interval*i*normal(0);
        point.y = disturb_pt(1)+interval*i*normal(1);//vertical down
        point.z = disturb_pt(2)+interval*i*normal(2);
        kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        distance_down.push_back(pointNKNSquaredDistance[0]);
    }

}

void cal_nearest_pt_random(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f normal, Eigen::Vector3f pt, float offset, std::vector<float>& distance_down){
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);
    int step = 100;
    float move = 5.0;
    move = move+offset;
    float interval = 2.0; //direction : 0,1,0 baecause in this house, vertical axis is y-axis.

    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    Eigen::Vector3f disturb_pt = pt + move*normal;

    srand((unsigned)time(NULL));
    float result_0 = rand() / float(RAND_MAX);
    srand((unsigned)4*time(NULL));
    float result_1 = rand() / float(RAND_MAX);
    srand((unsigned)8*time(NULL));
    float result_2 = rand() / float(RAND_MAX);

    Eigen::Vector3f random_d(result_0, result_1, result_2);
    random_d /= random_d.norm();

    for(int i=0;i<step;i++){
        pcl::PointXYZ point;
        point.x = disturb_pt(0)+interval*i*random_d(0);
        point.y = disturb_pt(1)+interval*i*random_d(1);//vertical down
        point.z = disturb_pt(2)+interval*i*random_d(2);
        kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        distance_down.push_back(pointNKNSquaredDistance[0]);
    }

}

void judge_i_o(std::vector<float> distance_list_down, int& polar){
    float dis_th = 1.5;
    float gap_th = 5.0;
    float ground = -53.0;
    int gap = 4;
    int gap_id = 5;
    int polar_index = 0;
    for (int i=gap; i<distance_list_down.size()-gap; i++){
        if ((distance_list_down[i-gap] > distance_list_down[i]) && (distance_list_down[i+gap] > distance_list_down[i]) && (distance_list_down[i] < dis_th) && (distance_list_down[i-gap]-distance_list_down[i] > gap_th) && (distance_list_down[i+gap]-distance_list_down[i] > gap_th) && ((i-polar_index) >= gap_id)){
            polar += 1;
            polar_index = i;
        }
    }
    float minValue = *min_element(distance_list_down.begin(),distance_list_down.end());
    int minPosition = min_element(distance_list_down.begin(),distance_list_down.end()) - distance_list_down.begin();

    if ((polar == 0) && minValue < 1.0 && minPosition != 0){
        polar += 1;
    }
}

// void neighbor_judge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)

int main()
{
    std::string pcd_name = "/home/albert/obj_model/house_obj/house/textures/house_fill_p.pcd";
    std::string test_pcd = "/home/albert/pointcloud_process/kmeans/build/b-24.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr after_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_name,*process_cloud);
    std::ofstream OutFile("/home/albert/pointcloud_process/kmeans/build/pc_test.txt");

    float search_radius = 5.0;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (process_cloud);
    ne.setViewPoint(0.0, 200.0, 0.0);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (search_radius);
    ne.compute (*cloud_normals);

    int undiscri = 0;
    int end = process_cloud->points.size();
    srand((unsigned)time(NULL));
    int test = 250;//(rand() % (end+1));

    for (int i=0;i<end;i = i+50)
    {
    int process_id = i;
    if(process_cloud->points[process_id].y > -40.0){
    std::vector<int> neighberID;

    knn(process_cloud, process_id, neighberID);

    Eigen::Vector3f normal_pt;
    Eigen::Vector3f focus_pt(process_cloud->points[i].x, process_cloud->points[i].y, process_cloud->points[i].z);
    Eigen::Vector3f waiting_1(cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z);
    Eigen::Vector3f waiting_2 = -1*waiting_1;
    

    // std::cout << "vec_1: " << waiting_1 << std::endl;
    // std::cout << "vec_2: " << waiting_2 << std::endl;

    std::vector<float> surf_distance_1;
    int polar_1 = 0;
    std::vector<float> surf_distance_2;
    int polar_2 = 0;

    cal_nearest_pt(process_cloud, waiting_1, focus_pt, surf_distance_1);
    judge_i_o(surf_distance_1, polar_1);

    cal_nearest_pt(process_cloud, waiting_2, focus_pt, surf_distance_2);
    judge_i_o(surf_distance_2, polar_2);

    std::vector<float> v_1;
    int cal = 0;
    float mp = 0.0;

    while ((polar_1%2 == polar_2%2)){
        if (cal <= 500){
        polar_1 = 0;
        polar_2 = 0;
        surf_distance_1.clear();
        surf_distance_2.clear();

        cal_nearest_pt_random(process_cloud, waiting_1, focus_pt, mp, surf_distance_1);
        judge_i_o(surf_distance_1, polar_1);
        cal_nearest_pt_random(process_cloud, waiting_2, focus_pt, mp, surf_distance_2);
        judge_i_o(surf_distance_2, polar_2);
        }
        else if (cal>=500 && cal<1000){
            mp = 45.0;
            polar_1 = 0;
            polar_2 = 0;
            surf_distance_1.clear();
            surf_distance_2.clear();

            cal_nearest_pt_random(process_cloud, waiting_1, focus_pt, mp, surf_distance_1);
            judge_i_o(surf_distance_1, polar_1);
            cal_nearest_pt_random(process_cloud, waiting_2, focus_pt, mp, surf_distance_2);
            judge_i_o(surf_distance_2, polar_2);
        }
        else if(cal>=1000){
            undiscri += 1;
            std::cout << "id: " << i << std::endl;
            break;
        }
        cal += 1;
    }

    for(int i=0; i<surf_distance_1.size();i++){
      std::cout << "dis: "<<surf_distance_1[i] << std::endl;
    }
    std::cout<<"--------------------------------------------------- "<<std::endl;

    for(int i=0; i<surf_distance_2.size();i++){
      std::cout << "dis: "<<surf_distance_2[i] << std::endl;
    }
    std::cout<<"polar_1: "<<polar_1<<std::endl;
    std::cout<<"polar_2: "<<polar_2<<std::endl;
    std::cout << "cal: " << cal << std::endl;
    // std::cout << "id: " << i << std::endl;

    v_1.push_back(focus_pt(0));
    v_1.push_back(focus_pt(1));
    v_1.push_back(focus_pt(2));

    if ((polar_1 % 2 == 0) && (polar_2 % 2 == 1)){
        v_1.push_back(waiting_1(0));
        v_1.push_back(waiting_1(1));
        v_1.push_back(waiting_1(2));
    }
    else if(polar_2 == 0){
        v_1.push_back(waiting_2(0));
        v_1.push_back(waiting_2(1));
        v_1.push_back(waiting_2(2));
    }
    else if(polar_1 == 0){
        v_1.push_back(waiting_1(0));
        v_1.push_back(waiting_1(1));
        v_1.push_back(waiting_1(2));
    }
    else if(polar_2 % 2 == 1){
            v_1.push_back(waiting_1(0));
            v_1.push_back(waiting_1(1));
            v_1.push_back(waiting_1(2));
    }
    else if(polar_1 % 2 == 1){
            v_1.push_back(waiting_2(0));
            v_1.push_back(waiting_2(1));
            v_1.push_back(waiting_2(2));
    }
    else{
        v_1.push_back(waiting_2(0));
        v_1.push_back(waiting_2(1));
        v_1.push_back(waiting_2(2));
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////

    std::stringstream ce;
    std::copy(v_1.begin(), v_1.end(), std::ostream_iterator<float>(ce, " "));

    OutFile << ce.str().c_str();
    OutFile << std::endl;
    }
    }
    OutFile.close();
    std::cout<<"test_id: "<<test<<std::endl;

    return 0;
    
}