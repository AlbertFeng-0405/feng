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


int main(int argc,char **argv){
    //ofstream OutFile("/home/albert/learn_cpp/kmeans/build/center.txt");
    
    for(int i=1; i<=50; i++){
    std::vector<double> center;
    std::string cloud_name = "/home/albert/learn_cpp/kmeans/build/b-"+std::to_string(i)+".pcd";

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

    Eigen::Vector3f v(vec(0, ii), vec(1, ii), vec(2, ii));
    v /= v.norm();

    std::cout<< "Principlal direction:" << v << std::endl;

    std::stringstream result;
    std::copy(center.begin(), center.end(), std::ostream_iterator<float>(result, " "));
    // OutFile << std::to_string(i)+" ";
    // OutFile << result.str().c_str();
    // OutFile << std::endl;
    
    }
    //OutFile.close();

    return 0;
}