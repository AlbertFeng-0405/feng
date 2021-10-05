#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <cstdlib>

bool judge_in_out(Eigen::Vector3d v_1, Eigen::Vector3d v_2, Eigen::Vector3d v_ref, std::vector<double> center_pt){
    Eigen::Vector3d c_cluster(center_pt[0], center_pt[1], center_pt[2]);
    Eigen::Vector3d dis_1 = c_cluster + 4*v_1;
    Eigen::Vector3d dis_2 = c_cluster + 4*v_2;
    //dis_1.norm() vs. dis_2.norm()

    double inner_1 = v_1.dot(v_ref) /(v_1.norm()*v_ref.norm());
    double angleNew_1 = acos(inner_1) * 180 / M_PI;
    //std::cout << "angle_1:" << angleNew_1 << std::endl;

    double inner_2 = v_2.dot(v_ref) /(v_2.norm()*v_ref.norm());
    double angleNew_2 = acos(inner_2) * 180 / M_PI;
    //std::cout << "angle_2:" << angleNew_2 << std::endl;

    if (angleNew_1<=angleNew_2){
        return true;
    }
    if (angleNew_1>angleNew_2){
        return false;
    }
}

void getpts(std::string path, std::vector<double>& lines){
	ifstream infile_feat(path);
    std::string feature; //存储读取的每行数据
	float feat_onePoint;  //存储每行按空格分开的每一个float数据
	lines.clear();

	while(!infile_feat.eof()) 
	{	
		getline(infile_feat, feature); //一次读取一行数据
		std::stringstream ss(feature); //使用串流实现对string的输入输出操作
		while (ss >> feat_onePoint) {      //按空格一次读取一个数据存入feat_onePoint 
			lines.push_back(feat_onePoint); //存储每行按空格分开的数据 
		}
	}
	infile_feat.close();
}

float sampling_config(Eigen::Vector3d normal, std::vector<double> center_position, int seed){
    Eigen::Vector2f direction(normal(0),normal(2));
    Eigen::Vector2f sample_vec;
    
    float s_x, s_z, yaw; //because up axis is y_axis
    srand(seed); 
    s_x = (float)50*rand()/float(RAND_MAX)-25;//sample range 50 x 50 m^2
    s_z = (float)50*rand()/float(RAND_MAX)-25;
    yaw = (rand() / float(RAND_MAX))*2*M_PI;

    sample_vec(0) = s_x-direction(0);
    sample_vec(1) = s_z-direction(1);
    while (sample_vec.norm()>4.5 || (acos(direction.dot(sample_vec) /(direction.norm()*sample_vec.norm())) * 180 / M_PI)>30.0){
      seed += 1;
      srand(seed); 
      s_x = (float)50*rand()/float(RAND_MAX)-25;//sample range 50 x 50 m^2
      s_z = (float)50*rand()/float(RAND_MAX)-25;
      yaw = (rand() / float(RAND_MAX))*2*M_PI;
    }

    return s_x, s_z, yaw;
}

bool judge_in_frustum(Eigen::Vector3f pc_pt, Eigen::Vector3f config, float yaw_angle){// judege if point from this cluster can be seen by UAV
    Eigen::Vector3f fov_normal(cos(yaw_angle), 0.0, sin(yaw_angle));
    Eigen::Vector3f check_vec = pc_pt - config;

    float included_angle = acos(check_vec.dot(fov_normal) /(check_vec.norm()*fov_normal.norm())) * 180 / M_PI;
    
    if (check_vec.norm()<6.0 && included_angle<30.0){
        return true;
    }
    return false;
}

int main(int argc,char **argv){
    //ofstream OutFile("/home/albert/learn_cpp/kmeans/build/center.txt");
    Eigen::Vector3d z_direction(0.0,1.0,0.0); // for this house.obj, phisical z_axis is y_axis in model. Temp pending for demo test.
    std::vector<double> ref_vec;
    std::string ref_path;
	ref_path = "/home/albert/pointcloud_process/kmeans/build/ref.txt";
    getpts(ref_path, ref_vec);

    int cluster = 50;
    float threshold_rate = 0.85;

    for(int i=1; i<=cluster; i++){
    std::vector<double> center;
    Eigen::Vector3d refv(ref_vec[3*(i-1)], ref_vec[3*(i-1)+1], ref_vec[3*(i-1)+2]);

    std::string cloud_name = "/home/albert/pointcloud_process/kmeans/build/b-"+std::to_string(i)+".pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_name,*cloud);

    int pt_num = cloud->size();
    double x_sum = 0.0;
    double y_sum = 0.0;
    double z_sum = 0.0;

    int rows = 3;
    Eigen::MatrixXd tmp_mat;
    tmp_mat.resize(rows,pt_num);//dynamic matrix

    for(int i=0;i<cloud->points.size();i++){
        x_sum += cloud->points[i].x;
        y_sum += cloud->points[i].y;
        z_sum += cloud->points[i].z;

    }

    x_sum = x_sum/pt_num;
    center.push_back(x_sum);
    y_sum = y_sum/pt_num;
    center.push_back(y_sum);
    z_sum = z_sum/pt_num;
    center.push_back(z_sum);
    for(int i=0;i<cloud->points.size();i++){
      tmp_mat(0,i) = (cloud->points[i].x - x_sum);
      tmp_mat(1,i) = (cloud->points[i].y - y_sum);
      tmp_mat(2,i) = (cloud->points[i].z - z_sum);
    }
    Eigen::Matrix3d H = tmp_mat * tmp_mat.transpose();
    
    Eigen::EigenSolver<Eigen::Matrix3d> es(H); // PCA
    Eigen::Matrix3d val = es.pseudoEigenvalueMatrix();
    Eigen::Matrix3d vec = es.pseudoEigenvectors();
    
    double t1 = val(0, 0);
    int ii = 0;
    if (t1 > val(1, 1)) {
        ii = 1;
        t1 = val(1, 1);
    }
    if (t1 > val(2, 2)){
        ii = 2;
        t1 = val(2, 2);
    }

    Eigen::Vector3d v(vec(0, ii), vec(1, ii), vec(2, ii));
    v /= v.norm();

    Eigen::Vector3d waiting_1 = z_direction.cross(v);
    Eigen::Vector3d waiting_2 = v.cross(z_direction);

    bool flag;
    flag = judge_in_out(waiting_1, waiting_2, refv, center);
    Eigen::Vector3d face_normal;
    if (flag){
        face_normal = waiting_1;
    }
    if (!flag){
        face_normal = waiting_2;
    }// decide the normal of this cluster

    std::cout << "normal:" << face_normal << std::endl;

    float config_y = center[1];
    float config_x, config_z, config_yaw, including_rate;
    config_x, config_z, config_yaw = sampling_config(face_normal, center, i);
    Eigen::Vector3f config(config_x, config_y, config_z);
    int inners = 0;
    for(int i=0;i<cloud->points.size();i++){
        Eigen::Vector3f pc_point(cloud->points[i].z, cloud->points[i].y, cloud->points[i].z);
        if (judge_in_frustum(pc_point, config, config_yaw)){
            inners += 1;
        }
    }
    including_rate = (float)inners/pt_num;


    int epoch = 1;
    while (including_rate > threshold_rate){
        config_x, config_z, config_yaw = sampling_config(face_normal, center, i+epoch*cluster);
        config(0) = config_x;
        config(2) = config_z;
        inner = 0;
        for(int i=0;i<cloud->points.size();i++){
        Eigen::Vector3f pc_point(cloud->points[i].z, cloud->points[i].y, cloud->points[i].z);
        if (judge_in_frustum(pc_point, config, config_yaw)){
            inners += 1;
        }
    }
    including_rate = (float)inners/pt_num;
    }

    std::cout << "configration:" << config << std::endl;
    std::cout << "yaw_angle:" << config_yaw << std::endl;

    //std::cout<< "Principlal direction:" << v << std::endl;

    std::stringstream result;
    std::copy(center.begin(), center.end(), std::ostream_iterator<float>(result, " "));
    // OutFile << std::to_string(i)+" ";
    // OutFile << result.str().c_str();
    // OutFile << std::endl;
    
    }
    //OutFile.close();

    return 0;
}