#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <string>

bool
customRegionGrowing (const pcl::PointXYZINormal& seedPoint, const pcl::PointXYZINormal& candidatePoint, float squared_distance)
{
  Eigen::Vector3d N1(
		seedPoint.normal_x,
		seedPoint.normal_y,
		seedPoint.normal_z
	);
	Eigen::Vector3d N2(
		candidatePoint.normal_x,
		candidatePoint.normal_y,
		candidatePoint.normal_z
	);
 
    Eigen::Vector3d D1(
        seedPoint.x,
        seedPoint.y,
        seedPoint.z
    );
    Eigen::Vector3d D2(
        candidatePoint.x,
        candidatePoint.y,
        candidatePoint.z
    );
    Eigen::Vector3d Dis = D1-D2;

	double angle = acos(N1.dot(N2)/N1.norm()/N2.norm());
    double distance = Dis.norm();

	const double threshold_angle = 2.0;	//[deg]
	if((angle/M_PI*180.0 < threshold_angle))	return true;//&& (distance < 10.0)
	else	return false;
}


int main(int argc, char** argv)
{
    std::string pcd_name = "/home/albert/obj_model/house_obj/house/textures/house_fill_p.pcd";
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fill_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr after_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inner_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
    pcl::console::TicToc tt;

    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_name,*fill_cloud);
    
    for (int i=0; i<fill_cloud->points.size(); ++i){
        if (fill_cloud->points[i].y > -47.0){
            process_cloud->push_back(fill_cloud->points[i]);
        }
    }

    pcl::copyPointCloud (*process_cloud, *after_cloud);

    float search_radius = 5.0;

    pcl::copyPointCloud (*after_cloud, *cloud_normals);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
    ne.setInputCloud (after_cloud);
    //ne.setViewPoint(0.0, 200.0, 0.0);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (search_radius);
    ne.compute (*cloud_normals);

    std::cerr << "Segmenting to clusters...\n", tt.tic ();
    pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec (true);
    cec.setInputCloud (cloud_normals);
    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (1.7);
    cec.setMinClusterSize (cloud_normals->size () / 1000);
    cec.setMaxClusterSize (cloud_normals->size () / 5);
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);
    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    std::cout<< clusters->size() <<std::endl;
    std::cout<< small_clusters->size() <<std::endl;
    std::cout<< large_clusters->size() <<std::endl;

    for (int i = 0; i < small_clusters->size (); ++i)
    for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
      (*after_cloud)[(*small_clusters)[i].indices[j]].intensity = -2.0;
    for (int i = 0; i < large_clusters->size (); ++i)
      for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
        (*after_cloud)[(*large_clusters)[i].indices[j]].intensity = +10.0;
    for (int i = 0; i < clusters->size (); ++i)
  {
    int label = rand () % 8;
    std::cout << ">> size: " << (*clusters)[i].indices.size() << "\n";
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
      (*after_cloud)[(*clusters)[i].indices[j]].intensity = label;
  }

    for (int i = 0; i < clusters->size (); ++i){//clusters->size ()
        for (int j = 0; j < (*clusters)[i].indices.size (); ++j){
            inner_cloud->push_back(after_cloud->points[(*clusters)[i].indices[j]]);
        }
    }

    pcl::io::savePCDFile ("plane.pcd", *inner_cloud);

    return (0);
}