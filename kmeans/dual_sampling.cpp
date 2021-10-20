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

void knn(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ point, std::vector<int>& pointIdxNKNSearch){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    int K = 1;
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
}

int main()
{
    std::string c_name = "/home/albert/pointcloud_process/kmeans/build/plane.pcd";
    std::string pcd_name = "/home/albert/obj_model/house_obj/house/textures/house_fill_p.pcd";
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZINormal>(c_name,*cluster_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_name,*process_cloud);

    std::vector<int> label_list;
    std::vector<int> set_label;
    for (int i=0;i<cluster_cloud->points.size();++i){
        label_list.push_back(cluster_cloud->points[i].intensity);
    }
    std::set<int> st(label_list.begin(), label_list.end());
    set_label.assign(st.begin(), st.end());
    
    for(int i=0;i<set_label.size();++i)
    {
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
        pcl::PointXYZ center_pt;
        center_pt.x = centroid(0);
        center_pt.y = centroid(1);
        center_pt.z = centroid(2);
        std::vector<int> near_id;
        knn(process_cloud, center_pt, near_id);
    }
}