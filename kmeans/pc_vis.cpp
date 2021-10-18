#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv) {
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_milk(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/albert/pointcloud_process/kmeans/build/vis.pcd", *cloud_milk);

    // 创建PCLVisualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // 设置背景色为灰色（非必须）
    viewer->setBackgroundColor (0.05, 0.05, 0.05, 0);

    // 再添加一个彩色点云及配置
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_milk);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_milk, rgb, "sample cloud milk");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud milk");

    // 直到窗口关闭才结束循环
    while (!viewer->wasStopped()) {
    
        // 每次循环调用内部的重绘函数
        viewer->spinOnce();  // 需要添加这行
    }
    return 0;
}