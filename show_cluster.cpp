#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
 
using namespace std;
 
int main()
{

//***************************read PCD file*****************************************

	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
    string addr = "/home/albert/learn_cpp/build/cluster/";
    string filename;
    int num = 31316;
    
    boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Viewer"));

    viewer->setBackgroundColor(0, 0, 0);

	int vp;
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);

    for(int i = 0;i < num; i++){
        filename = addr + "c_" + to_string(i) + ".pcd";
        pcl::io::loadPCDFile(filename, *source);

	    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source, 3*i, 4*i, 2*i);
 
	    viewer->addPointCloud<pcl::PointXYZ>(source, source_color, to_string(i), vp);
    
	    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    }
    viewer->addCoordinateSystem(1.0);
	viewer->spin();	
}