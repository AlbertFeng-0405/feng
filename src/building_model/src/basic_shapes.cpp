#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <cstdlib>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include </home/albert/planning_ws/src/building_model/include/basic_shapes.h>

void getpts(string path, vector<float>& lines){
	ifstream infile_feat(path);
  std::string feature; //存储读取的每行数据
	float feat_onePoint;  //存储每行按空格分开的每一个float数据
	lines.clear();

	while(!infile_feat.eof()) 
	{	
		getline(infile_feat, feature); //一次读取一行数据
		stringstream ss(feature); //使用串流实现对string的输入输出操作
		while (ss >> feat_onePoint) {      //按空格一次读取一个数据存入feat_onePoint 
			lines.push_back(feat_onePoint); //存储每行按空格分开的数据 
		}
	}
	infile_feat.close();
}

void get_bormal(geometry_msgs::Point normal_end, geometry_msgs::Point pt_0, geometry_msgs::Point pt_1, geometry_msgs::Point pt_2){
  Eigen::Vector3d v_0(pt_0.x-pt_1.x, pt_0.y-pt_1.y, pt_0.z-pt_1.z);
  Eigen::Vector3d v_1(pt_2.x-pt_1.x, pt_2.y-pt_1.y, pt_2.z-pt_1.z);
  Eigen::Vector3d v_n;
  v_n = v_0.cross(v_1);
  normal_end.x = v_n(0);
  normal_end.y = v_n(1);
  normal_end.z = v_n(2);

}

int main( int argc, char** argv )
{
  int cluster=50;
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  ros::Publisher normal_pub = n.advertise<visualization_msgs::Marker>("visualization_normal", 1);
  
  std::vector<float> c_and_n;
	std::string norm_path;
	norm_path = "/home/albert/optm_traj_reconstruction_demo/normal/norm.txt";
  getpts(norm_path, c_and_n);

  std::vector<string> name;
  for(int i=0;i<cluster;i++){
    string temp = "normal_"+to_string(i+1);
    name.push_back(temp);
  }

  std::vector<string> ns;
  for(int i=0;i<cluster;i++){
    string temp = "face_normal_"+to_string(i+1);
    ns.push_back(temp);
  }

  std::vector<float> norm_pts_s;
  std::vector<float> norm_pts_e;
 
  while (ros::ok())
  {
    for(int i=0;i<cluster;i++){
      visualization_msgs::Marker normal;
      
      normal.header.frame_id = "base_link";
      normal.header.stamp = ros::Time::now();
      normal.ns = "face_norm";
      normal.id = i+1;
      normal.type = visualization_msgs::Marker::ARROW;
      normal.action = visualization_msgs::Marker::ADD;

      normal.pose.position.x = 0;
      normal.pose.position.y = 0;
      normal.pose.position.z = 0;
      normal.pose.orientation.x = 0.0;
      normal.pose.orientation.y = 0.0;
      normal.pose.orientation.z = 0.0;
      normal.pose.orientation.w = 1.0;

      normal.scale.x = 0.2;
      normal.scale.y = 0.3;
      normal.scale.z = 0.1;

      normal.color.r = 0.0f;
      normal.color.g = 1.0f;
      normal.color.b = 0.0f;
      normal.color.a = 1.0;

      geometry_msgs::Point s;
      s.x = 0.0;//c_and_n[6*i];
      s.y = 0.0;//c_and_n[6*i+1];
      s.z = 0.0;//c_and_n[6*i+2];
      geometry_msgs::Point e;
      e.x = 0.0;//c_and_n[6*i]+c_and_n[6*i+3];
      e.y = 10.0;//c_and_n[6*i+1]+c_and_n[6*i+4];
      e.z = 0.0;//c_and_n[6*i+2]+c_and_n[6*i+5];

      normal.points.push_back(s);
      normal.points.push_back(e);
    
      normal.lifetime = ros::Duration();
      
      normal_pub.publish(normal);
    }

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
 
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
 
    // Set the marker type. 
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
 
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
 
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    // Set the color -- be sure to set alpha to something non-zero!
    
    for(int i=0;i<cluster;i++){
      srand(i); 
      double result_0 = rand() / double(RAND_MAX);
      double result_1 = rand() / double(RAND_MAX);
      double result_2 = rand() / double(RAND_MAX);
      std::vector<float> lines;
		  std::string file_path;
		  file_path = "/home/albert/optm_traj_reconstruction_demo/cluster/"+to_string(i)+".txt";
      getpts(file_path, lines);
      for(int j=0;j<lines.size();j=j+9){
        geometry_msgs::Point p1;
        p1.x = lines[j];
        p1.y = lines[j+1];
        p1.z = lines[j+2];
        geometry_msgs::Point p2;
        p2.x = lines[j+3];
        p2.y = lines[j+4];
        p2.z = lines[j+5];
        geometry_msgs::Point p3;
        p3.x = lines[j+6];
        p3.y = lines[j+7];
        p3.z = lines[j+8];
        
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        
        std_msgs::ColorRGBA c1;
        std_msgs::ColorRGBA c2;
        std_msgs::ColorRGBA c3;
        c1.a = 1;
        c1.r = result_0;
        c1.g = result_1;
        c1.b = result_2;
        c2.a = 1;
        c2.r = result_0;
        c2.g = result_1;
        c2.b = result_2;
        c3.a = 1;
        c3.r = result_0;
        c3.g = result_1;
        c3.b = result_2;
        
        marker.colors.push_back(c1);
        marker.colors.push_back(c2);
        marker.colors.push_back(c3);

      }
     

    }
 
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
 
    r.sleep();
  }

  return 0;
}