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
#include <algorithm>
#include <set>

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

void knn(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI point, std::vector<int>& pointIdxNKNSearch){
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (cloud);

    int K = 1;
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
}

void cal_eigen_direction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector4f centroid, Eigen::Vector3f& main_direction){
    int rows = 3;
    Eigen::MatrixXf tmp_mat;
    int pt_num = cloud->points.size();
    tmp_mat.resize(rows,pt_num);//dynamic matrix

    for(int i=0;i<cloud->points.size();i++){
      tmp_mat(0,i) = (cloud->points[i].x - centroid(0));
      tmp_mat(1,i) = (cloud->points[i].y - centroid(1));
      tmp_mat(2,i) = (cloud->points[i].z - centroid(2));
    }

    Eigen::Matrix3f H = tmp_mat * tmp_mat.transpose();
    
    Eigen::EigenSolver<Eigen::Matrix3f> es(H); // PCA
    Eigen::Matrix3f val = es.pseudoEigenvalueMatrix();
    Eigen::Matrix3f vec = es.pseudoEigenvectors();

    double t1 = val(0, 0);
    int ii = 0;
    if (t1 < val(1, 1)) {
        ii = 1;
        t1 = val(1, 1);
    }
    if (t1 < val(2, 2)){
        ii = 2;
        t1 = val(2, 2);
    }

    Eigen::Vector3f v(vec(0, ii), vec(1, ii), vec(2, ii));
    v /= v.norm();
    main_direction = v;
}

void cal_angle(Eigen::Vector3f direc, Eigen::Vector3f point, float& angle){
    float cosValNew=direc.dot(point) /(direc.norm()*point.norm()); 
    angle = acos(cosValNew) * 180 / M_PI;     
}

void cal_radius(pcl::PointCloud<pcl::PointXYZI>::Ptr cal_cloud, Eigen::Vector4f& centroid,float& r){					
	pcl::compute3DCentroid(*cal_cloud, centroid);
    Eigen::Vector4f max_pt;
    pcl::getMaxDistance(*cal_cloud, centroid, max_pt);// get most remote point in cloud and calculate the distance
    Eigen::Vector4f distance=max_pt-centroid;
    r = distance.norm();
}


int main(int argc, char** argv)
{
    std::string pcd_name = "/home/albert/obj_model/house_obj/house/textures/house_fill_p.pcd";
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fill_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr after_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inner_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
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
    ne.setViewPoint(0.0, 200.0, 0.0);
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

    std::cout<< clusters->size() <<std::endl;
    std::cout<< small_clusters->size() <<std::endl;
    std::cout<< large_clusters->size() <<std::endl;

    for (int i = 0; i < small_clusters->size (); ++i)
    for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
      (*after_cloud)[(*small_clusters)[i].indices[j]].intensity = -2.0;
    for (int i = 0; i < large_clusters->size (); ++i)
      for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
        (*after_cloud)[(*large_clusters)[i].indices[j]].intensity = +10.0;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr center_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> intensity_label;

    for (int i = 0; i < clusters->size (); i++)
  {
    int label = i+1;//rand () % 60;
    intensity_label.push_back(label);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    temp_cloud->clear();
    std::cout << ">> size: " << (*clusters)[i].indices.size() << "\n";
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j){
        (*after_cloud)[(*clusters)[i].indices[j]].intensity = label;
        temp_cloud->push_back(after_cloud->points[(*clusters)[i].indices[j]]);
    }
    Eigen::Vector4f centroid;					
	pcl::compute3DCentroid(*temp_cloud, centroid);

    pcl::PointXYZI point;
    point.x = centroid(0);
    point.y = centroid(1);//vertical down
    point.z = centroid(2);
    point.intensity = label;
    center_cloud->push_back(point);
  }

    for (int i = 0; i < clusters->size (); ++i){//clusters->size ()
        for (int j = 0; j < (*clusters)[i].indices.size (); ++j){
            inner_cloud->push_back(after_cloud->points[(*clusters)[i].indices[j]]);
        }
    }

    for (int i = 0; i<small_clusters->size();++i){
        for (int j=0;j<(*small_clusters)[i].indices.size();++j){
            corner_cloud->push_back(after_cloud->points[(*small_clusters)[i].indices[j]]);
        }
    }

    for(int i=0;i<corner_cloud->points.size();++i){
        std::vector<int> nearest_id;
        knn(center_cloud, corner_cloud->points[i], nearest_id);
        corner_cloud->points[i].intensity = center_cloud->points[nearest_id[0]].intensity;
        inner_cloud->push_back(corner_cloud->points[i]);
    }

    std::vector<int> real_label_list;
    for(int j=0; j<intensity_label.size();j++){
        std::cout<<"label!!!!!!!!!!!!!!!!!!!: " <<intensity_label[j]<<std::endl;
        int old_label = intensity_label[j];
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cluster_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        temp_cluster_cloud->clear();
        pcl::PointCloud<pcl::PointXYZI>::Ptr externel_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        externel_cloud->clear();
        std::vector<int> index;

        for(int i=0; i<inner_cloud->points.size();++i){
            if(inner_cloud->points[i].intensity == intensity_label[j]){
                temp_cluster_cloud->push_back(inner_cloud->points[i]);
                index.push_back(i);
            }
            else{
                externel_cloud->push_back(inner_cloud->points[i]);
            }
        }
        std::cout << "temp: " << temp_cluster_cloud->points.size() <<std::endl;
        std::cout << "temp: " << index.size() <<std::endl;
        Eigen::Vector4f cen;					
        float radius;
        cal_radius(temp_cluster_cloud, cen, radius);
        std::cout << "radius: " << radius <<std::endl;
    
        if(radius<15.0){
            std::vector<int> near_idx;
            pcl::PointXYZI point;
            point.x = cen(0);
            point.y = cen(1);
            point.z = cen(2);
            knn(externel_cloud, point, near_idx);
            for(int i=0;i<index.size();++i){
                inner_cloud->points[index[i]].intensity = externel_cloud->points[near_idx[0]].intensity;
            }
        }
        else{
            real_label_list.push_back(intensity_label[j]);
        }
        
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    for(int j=0; j<real_label_list.size();j++){
        std::cout<<"label!!!!!!!!!!!!!!!!!!!: " <<real_label_list[j]<<std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cluster_cloud_l (new pcl::PointCloud<pcl::PointXYZI>);
        temp_cluster_cloud_l->clear();
        std::vector<int> index_l;

        for(int i=0; i<inner_cloud->points.size();++i){
            if(inner_cloud->points[i].intensity == real_label_list[j]){
                temp_cluster_cloud_l->push_back(inner_cloud->points[i]);
                index_l.push_back(i);
            }
        }
        if (temp_cluster_cloud_l->points.size()>0){
            Eigen::Vector4f cen_l;			
            float radius_l;
            cal_radius(temp_cluster_cloud_l, cen_l, radius_l);
            std::cout << "radius: " << radius_l <<std::endl;
            if (radius_l>25.0){
                int new_label_1 = real_label_list[j]+109;
                int new_label_2 = real_label_list[j]+2211;
                Eigen::Vector3f direction;
                cal_eigen_direction(temp_cluster_cloud_l, cen_l, direction);
                for(int i=0;i<index_l.size();++i)
                {
                float temp_angle;
                Eigen::Vector3f pt_2_cen(inner_cloud->points[index_l[i]].x-cen_l(0), inner_cloud->points[index_l[i]].y-cen_l(1), inner_cloud->points[index_l[i]].z-cen_l(2));
                cal_angle(direction, pt_2_cen, temp_angle);
                if (temp_angle < 90.0){
                    inner_cloud->points[index_l[i]].intensity = new_label_1;
                }
                else{
                    inner_cloud->points[index_l[i]].intensity = new_label_2;
                }
                }
                real_label_list.push_back(new_label_1);
                real_label_list.push_back(new_label_2);
            }
        }
        
        index_l.clear();
        
    }
    std::cerr << ">> Done: " << tt.toc () << " ms\n";
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<int> new_label_list;
    for(int j=0; j<real_label_list.size();j++){
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cluster_cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
        temp_cluster_cloud_f->clear();
        std::vector<int> index_f;

        for(int i=0; i<inner_cloud->points.size();++i){
            if(inner_cloud->points[i].intensity == real_label_list[j]){
                temp_cluster_cloud_f->push_back(inner_cloud->points[i]);
                index_f.push_back(i);
            }
        }
        if (temp_cluster_cloud_f->points.size()>0){
            Eigen::Vector4f cen_f;			
            float radius_f;
            cal_radius(temp_cluster_cloud_f, cen_f, radius_f);
            std::cout << "radius: " << radius_f <<std::endl;
            new_label_list.push_back(real_label_list[j]);}
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    for(int j=0;j<new_label_list.size();++j){
        int r = (rand() % (255+1));
        int g = (rand() % (255+1));
        int b = (rand() % (255+1));
        for(int i=0; i<inner_cloud->points.size();++i){
            if(inner_cloud->points[i].intensity == new_label_list[j]){
                pcl::PointXYZRGB vis_point;
                vis_point.x=inner_cloud->points[i].x;
                vis_point.y=inner_cloud->points[i].y;
                vis_point.z=inner_cloud->points[i].z;
                vis_point.r=r;
                vis_point.g=g;
                vis_point.b=b;
                vis_cloud->push_back(vis_point);
            }
        }
    }

    std::cout<<"label: " <<intensity_label.size()<<std::endl;
    std::cout<<"real_label: " <<real_label_list.size()<<std::endl;
    std::cout<<"final_label: " <<new_label_list.size()<<std::endl;

    pcl::io::savePCDFile ("plane.pcd", *inner_cloud);
    pcl::io::savePCDFile ("corner.pcd", *corner_cloud);
    pcl::io::savePCDFile ("vis.pcd", *vis_cloud);

    return (0);
}