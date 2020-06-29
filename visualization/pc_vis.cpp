//
// Created by liuwch on 2020/5/25.
//
#include "../include/pc_vis.h"

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
    viewer.setBackgroundColor(1.0, 0.5, 1.0);   //设置背景颜色
}


void pc_vis(string &in_file) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    /*pcl::PCDReader reader;
    reader.read<pcl::PointXYZI>(in_file, *cloud);*/

    pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "x");					// 按照z字段进行渲染

    viewer->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");		// 设置点云大小

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    //pcl::visualization::CloudViewer viewer("Cloud Viewer");     //创建viewer对象
    //viewer.showCloud(cloud);
    //// viewer.runOnVisualizationThreadOnce(viewerOneOff);
    //while (!viewer.wasStopped()) {};
}