#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

void getpts(std::string path, std::vector<double>& lines){
	std::ifstream infile_feat(path);
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

int main(){
    std::vector<double> config_coord;
    std::string coord_path;
    coord_path = "/home/albert/pointcloud_process/kmeans/build/cfg.txt";// x_0, y_0, z_0, yaw_x_0, 0, yaw_z_0; ...; x_n, y_n, z_n, yaw_x_n, 0, yaw_z_n
    getpts(coord_path, config_coord);// get each position of Qref

    std::ofstream g_tsp("/home/albert/vtk/LKH-2.0.9/global.tsp");// tsp file for TSP solver
    int cluster = 50;

    g_tsp << "NAME : global\n";
    g_tsp << "TYPE : TSP\n";
    g_tsp << "DIMENSION : "+std::to_string(cluster) + "\n";
    g_tsp << "EDGE_WEIGHT_TYPE : EUC_3D\n";
    g_tsp << "NODE_COORD_SECTION\n";
    for(int i=0; i<cluster; i++){
        g_tsp << std::to_string(i+1)+" "+std::to_string(config_coord[6*i])+" "+std::to_string(config_coord[6*i+1])+" "+std::to_string(config_coord[6*i+2]);
        g_tsp << std::endl;
    }
    g_tsp << "EOF";
    g_tsp.close();

    std::ofstream g_par("/home/albert/vtk/LKH-2.0.9/global.par");// par file for TSP solver

    g_par << "PROBLEM_FILE = /home/albert/vtk/LKH-2.0.9/global.tsp\n";
    g_par << "GAIN23 = YES\n";
    g_par << "OUTPUT_TOUR_FILE = /home/albert/pointcloud_process/kmeans/global.txt\n";
    g_par << "RUNS = 10\n";
    g_par.close();
}