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

void cal_nearest_pt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f normal, Eigen::Vector3f pt, std::vector<float>& distance){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);
    int step = 100;
    float move = 50.0;
    float interval = 2.0; //direction : 0,1,0 baecause in this house, vertical axis is y-axis.

    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    Eigen::Vector3f disturb_pt = pt + move*normal;

    for(int i=0;i<step;i++){
        pcl::PointXYZ point;
        point.x = disturb_pt(0);
        point.y = disturb_pt(1)+interval*i;
        point.z = disturb_pt(2);
        // std::cout<< "x: " << point.x << std::endl;
        // std::cout<< "y: " << point.y << std::endl;
        // std::cout<< "z: " << point.z << std::endl;
        kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        distance.push_back(pointNKNSquaredDistance[0]);
    }

}

int main()
{
    std::string pcd_name = "/home/albert/obj_model/house_obj/house/textures/house.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_name,*process_cloud);

    int process_id = 1;
    std::vector<int> neighberID;

    knn(process_cloud, process_id, neighberID);

    Eigen::Vector3f waiting_1;
    Eigen::Vector3f waiting_2;
    Eigen::Vector3f focus_pt(process_cloud->points[process_id].x, process_cloud->points[process_id].y, process_cloud->points[process_id].z);

    cal_2_norm(process_cloud, neighberID, waiting_1, waiting_2);

    std::cout << "vec_1: " << waiting_1 << std::endl;
    std::cout << "vec_2: " << waiting_2 << std::endl;

    std::vector<float> surf_distance;

    cal_nearest_pt(process_cloud, waiting_1, focus_pt, surf_distance);
    std::cout<<"distance_num: "<<surf_distance.size()<<std::endl;
    for (int i=0; i<surf_distance.size(); i++){
        std::cout<<"distance: "<<surf_distance[i]<<std::endl;
    }

    return 0;
    
}