#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>	

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <string>
#include <vector>
#include <set>
#include <ctime>
#include <time.h>

void knn(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointNormal point, std::vector<int>& pointIdxNKNSearch){
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud (cloud);

    int K = 1;
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
}

void cal_nearest_pt(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::Vector3f normal, Eigen::Vector3f pt, std::vector<float>& distance_down)
{
    
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud (cloud);
    int step = 100;
    float move = 5.0;
    float interval = 2.0; //direction : 0,1,0 baecause in this house, vertical axis is y-axis.

    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    Eigen::Vector3f disturb_pt = pt + move*normal;

    for(int i=0;i<step;i++){
        pcl::PointNormal point;
        point.x = disturb_pt(0)+interval*i*normal(0);
        point.y = disturb_pt(1)+interval*i*normal(1);//vertical down
        point.z = disturb_pt(2)+interval*i*normal(2);
        kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        distance_down.push_back(pointNKNSquaredDistance[0]);
    }

}

void cal_nearest_pt_random(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::Vector3f normal, Eigen::Vector3f pt, float offset, std::vector<float>& distance_down)
{
    
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
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
        pcl::PointNormal point;
        point.x = disturb_pt(0)+interval*i*random_d(0);
        point.y = disturb_pt(1)+interval*i*random_d(1);//vertical down
        point.z = disturb_pt(2)+interval*i*random_d(2);
        kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        distance_down.push_back(pointNKNSquaredDistance[0]);
    }

}

void judge_i_o(std::vector<float> distance_list_down, int& polar)
{
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

void judge_func(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int idx, std::vector<float>& center_nm)
{
    Eigen::Vector3f focus_pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
    Eigen::Vector3f waiting_1(cloud->points[idx].normal_x, cloud->points[idx].normal_y, cloud->points[idx].normal_z);
    Eigen::Vector3f waiting_2 = -1*waiting_1;

    std::vector<float> surf_distance_1;
    int polar_1 = 0;
    std::vector<float> surf_distance_2;
    int polar_2 = 0;

    cal_nearest_pt(cloud, waiting_1, focus_pt, surf_distance_1);
    judge_i_o(surf_distance_1, polar_1);

    cal_nearest_pt(cloud, waiting_2, focus_pt, surf_distance_2);
    judge_i_o(surf_distance_2, polar_2);

    int cal = 0;
    float mp = 0.0;

    while ((polar_1%2 == polar_2%2)){
        if (cal <= 500){
        polar_1 = 0;
        polar_2 = 0;
        surf_distance_1.clear();
        surf_distance_2.clear();

        cal_nearest_pt_random(cloud, waiting_1, focus_pt, mp, surf_distance_1);
        judge_i_o(surf_distance_1, polar_1);
        cal_nearest_pt_random(cloud, waiting_2, focus_pt, mp, surf_distance_2);
        judge_i_o(surf_distance_2, polar_2);
        }
        else if (cal>=500 && cal<800){
            mp = 45.0;
            polar_1 = 0;
            polar_2 = 0;
            surf_distance_1.clear();
            surf_distance_2.clear();

            cal_nearest_pt_random(cloud, waiting_1, focus_pt, mp, surf_distance_1);
            judge_i_o(surf_distance_1, polar_1);
            cal_nearest_pt_random(cloud, waiting_2, focus_pt, mp, surf_distance_2);
            judge_i_o(surf_distance_2, polar_2);
        }
        else if(cal>=800){
            break;
        }
        cal += 1;
    }

    center_nm.push_back(focus_pt(0));
    center_nm.push_back(focus_pt(1));
    center_nm.push_back(focus_pt(2));

    if ((polar_1 % 2 == 0) && (polar_2 % 2 == 1)){
        center_nm.push_back(waiting_1(0));
        center_nm.push_back(waiting_1(1));
        center_nm.push_back(waiting_1(2));
    }
    else if(polar_2 == 0){
        center_nm.push_back(waiting_2(0));
        center_nm.push_back(waiting_2(1));
        center_nm.push_back(waiting_2(2));
    }
    else if(polar_1 == 0){
        center_nm.push_back(waiting_1(0));
        center_nm.push_back(waiting_1(1));
        center_nm.push_back(waiting_1(2));
    }
    else if(polar_2 % 2 == 1){
            center_nm.push_back(waiting_1(0));
            center_nm.push_back(waiting_1(1));
            center_nm.push_back(waiting_1(2));
    }
    else if(polar_1 % 2 == 1){
            center_nm.push_back(waiting_2(0));
            center_nm.push_back(waiting_2(1));
            center_nm.push_back(waiting_2(2));
    }
    else{
        center_nm.push_back(waiting_2(0));
        center_nm.push_back(waiting_2(1));
        center_nm.push_back(waiting_2(2));
    }
}

int main()
{
    std::string c_name = "/home/albert/pointcloud_process/kmeans/build/plane.pcd";
    std::string pcd_name = "/home/albert/obj_model/house_obj/house/textures/house_fill_p.pcd";
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>(c_name,*cluster_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_name,*process_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    float search_radius = 2.5;
    pcl::copyPointCloud (*process_cloud, *cloud_normals);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud (process_cloud);
    ne.setViewPoint(0.0, 200.0, 0.0);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (search_radius);
    ne.compute (*cloud_normals);

    std::vector<int> label_list;
    std::vector<int> set_label;
    for (int i=0;i<cluster_cloud->points.size();++i){
        label_list.push_back(cluster_cloud->points[i].intensity);
    }
    std::set<int> st(label_list.begin(), label_list.end());
    set_label.assign(st.begin(), st.end());
    
    for(int i=0;i<set_label.size();++i){
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
        temp_cloud->clear();
        for(int j=0; j<cluster_cloud->points.size();++j)
        {
            if (cluster_cloud->points[j].intensity == set_label[i])
            {
                temp_cloud->push_back(cluster_cloud->points[i]);
            }
        }
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*temp_cloud, centroid);
        pcl::PointNormal center_pt;
        center_pt.x = centroid(0);
        center_pt.y = centroid(1);
        center_pt.z = centroid(2);
        std::vector<int> near_id;
        knn(cloud_normals, center_pt, near_id);
        
        std::vector<float> center_normal;
        judge_func(cloud_normals, near_id[0], center_normal);// center_normal means the center and normal of this cluster.
    }
}