#include <fstream>
#include <iostream>
#include <vector>
//using namespace std;
#include <sstream>
#include <string>
#include </home/albert/planning_ws/src/building_model/include/test_read.h>

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

int main()
{
    for(int j=0;j<5;j++){
		float result = (float)j/5.0;
		std::vector<float> lines;
		std::string file_path;
		file_path = "/home/albert/optm_traj_reconstruction_demo/cluster/"+to_string(j)+".txt";
        getpts(file_path, lines);
		cout<<lines.size()<<endl;
		cout<<result<<endl;
		// for(int j=0;j<lines.size();j+9){
		// 	cout<<lines[j]<<endl;
		// 	cout<<j<<endl;
		// }
	}
	
	return 0;

}