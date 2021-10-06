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
#include <ctime>

bool judge_in_out(Eigen::Vector3d v_1, Eigen::Vector3d v_2, Eigen::Vector3d v_ref, std::vector<double> center_pt){
    Eigen::Vector3d remote(0.0, -500.0, 0.0);// 500 important
    Eigen::Vector3d up(0.0, 1.0, 0.0);
    Eigen::Vector3d c_cluster(center_pt[0], center_pt[1], center_pt[2]);
    Eigen::Vector3d dis_1 = c_cluster + 50*v_1;
    Eigen::Vector3d dis_2 = c_cluster + 50*v_2;// 50 important disturbance
    Eigen::Vector3d w_1 = dis_1-remote;
    Eigen::Vector3d w_2 = dis_2-remote;
    //dis_1.norm() vs. dis_2.norm()

    double inner_1 = w_1.dot(up) /(w_1.norm()*up.norm());
    double angleNew_1 = acos(inner_1) * 180 / M_PI;
    //std::cout << "angle_1:" << angleNew_1 << std::endl;

    double inner_2 = w_2.dot(up) /(w_2.norm()*up.norm());
    double angleNew_2 = acos(inner_2) * 180 / M_PI;
    //std::cout << "angle_2:" << angleNew_2 << std::endl;

    if (angleNew_1 > angleNew_2){//angleNew_1<=angleNew_2 | (dis_1-remote).norm()>(dis_2-remote).norm()
        return true;
    }
    if (angleNew_1 <= angleNew_2){//angleNew_1>angleNew_2 | (dis_2-remote).norm()>(dis_1-remote).norm()
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

void sampling_config(Eigen::Vector3d normal, std::vector<double>& center_position, int seed, float& s_x, float& s_z, float& yaw_x, float& yaw_z){
    Eigen::Vector2f direction(normal(0),normal(2));
    float norm_length, norm_angle, direc_angle, s_l, s_r;
    float frustum_range = 7.0, min_range = 2.0;
    float frustum_angle = M_PI/6;

    direc_angle = atan(normal(2)/normal(0));
    s_l = direc_angle - frustum_angle;
    s_r = direc_angle + frustum_angle;
    
    srand (static_cast <unsigned> (seed));
    norm_length = min_range + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(frustum_range-min_range)));
    srand (static_cast <unsigned> (4*seed));
    norm_angle =  s_l + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(s_r-s_l)));

    Eigen::Vector2f sample_vec(norm_length*cos(norm_angle), norm_length*sin(norm_angle));
    
    s_x = center_position[0] + sample_vec(0);
    s_z = center_position[2] + sample_vec(1);
    yaw_x = -sample_vec(0);
    yaw_z = -sample_vec(1);
}

bool judge_in_frustum(Eigen::Vector3f pc_pt, Eigen::Vector3f config, float yaw_x, float yaw_z){// judege if point from this cluster can be seen by UAV
    Eigen::Vector3f fov_normal(yaw_x, 0.0, yaw_z);
    Eigen::Vector3f check_vec = pc_pt - config;
    Eigen::Vector2f check_dis(check_vec(0), check_vec(2));
    float camera_range = 50.0;
    float fov_range = 30.0;

    float included_angle = acos(check_vec.dot(fov_normal) /(check_vec.norm()*fov_normal.norm())) * 180 / M_PI;
    // std::cout<< "dis: " << check_dis.norm();
    // std::cout<< "ang: " << included_angle;
    
    if (included_angle<fov_range){//&& check_dis.norm()<camera_range
        return true;
    }
    else{
        return false;
    }
}


int main(int argc,char **argv){
    ofstream OutFile("/home/albert/pointcloud_process/kmeans/build/normal.txt");
    std::string cfg_path = "/home/albert/pointcloud_process/kmeans/build/cfg.txt";
    std::ofstream file(cfg_path);

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
    Eigen::Vector3d v_ = -v;

    // Eigen::Vector3d waiting_1 = z_direction.cross(v);
    // Eigen::Vector3d waiting_2 = v.cross(z_direction);

    bool flag;
    flag = judge_in_out(v, v_, refv, center);
    Eigen::Vector3d face_normal;
    if (flag){
        face_normal = v;
    }
    if (!flag){
        face_normal = v_;
    }// decide the normal of this cluster

    face_normal(1) = 0.0;// because y_axis is vertical to ground
    face_normal /= face_normal.norm();

    //std::cout << "normal:" << face_normal << std::endl;
    std::vector<float> normal_vector;
    normal_vector.push_back(face_normal(0));
    normal_vector.push_back(face_normal(1));
    normal_vector.push_back(face_normal(2));

    float config_y = center[1];
    float config_x, config_z, config_yaw_x, config_yaw_z, including_rate;

    sampling_config(face_normal, center, i, config_x, config_z, config_yaw_x, config_yaw_z);

    Eigen::Vector3f con(config_x, config_y, config_z);
    Eigen::Vector3f con_view(config_yaw_x, 0.0, config_yaw_z);
    con_view /= con_view.norm();

    std::cout << "configration:" << con << std::endl;
    int inners = 0;
    for(int i=0;i<cloud->points.size();i++){
        Eigen::Vector3f pc_point(cloud->points[i].z, cloud->points[i].y, cloud->points[i].z);
        if (judge_in_frustum(pc_point, con, config_yaw_x, config_yaw_z)){
            inners += 1;
        }
    }
    including_rate = (float)inners/pt_num;
    std::cout << "rate:" << including_rate << std::endl;


    // int epoch = 1;
    // while (including_rate < threshold_rate){
    //     sampling_config(face_normal, center, i+epoch*2, config_x, config_z, config_yaw_x, config_yaw_z);
    //     con(0) = config_x;
    //     con(2) = config_z;
    //     inners = 0;
    //     for(int i=0;i<cloud->points.size();i++){
    //     Eigen::Vector3f pc_point(cloud->points[i].z, cloud->points[i].y, cloud->points[i].z);
    //     if (judge_in_frustum(pc_point, con, config_yaw_x, config_yaw_z)){
    //         inners += 1;
    //     }
    // }
    // including_rate = (float)inners/pt_num;
    // epoch +=1;
    // // std::cout << "update_rate: " << including_rate << std::endl;
    // }

    // std::cout << "configration:" << con << std::endl;
    
    std::stringstream ce;
    std::copy(center.begin(), center.end(), std::ostream_iterator<float>(ce, " "));
    std::stringstream result;
    std::copy(normal_vector.begin(), normal_vector.end(), std::ostream_iterator<float>(result, " "));
    //OutFile << std::to_string(i)+" ";
    OutFile << ce.str().c_str();
    OutFile << result.str().c_str();
    OutFile << std::endl;
    
    std::stringstream cfg;
    std::vector<float> cfg_space;
    cfg_space.push_back(con(0));
    cfg_space.push_back(con(1));
    cfg_space.push_back(con(2));
    cfg_space.push_back(con_view(0));
    cfg_space.push_back(con_view(1));
    cfg_space.push_back(con_view(2));
    std::copy(cfg_space.begin(), cfg_space.end(), std::ostream_iterator<float>(cfg, " "));

    file << cfg.str().c_str();
    file << std::endl;
    }
    OutFile.close();
    file.close();

    return 0;
}