//
// Created by liuwch on 2020/5/24.
//
#include "../include/pcd2bin.h"

void pcd2bin(string& in_file, string& out_file) {
    std::ofstream output(out_file.c_str(), std::ios::binary);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCDReader reader;
    reader.read(in_file, *points);

    for (int i = 0; i < points->width; i++) {
        output.write((char*)(&points->points[i].x), 3 * sizeof(float));
        output.write((char*)&points->points[i].intensity, sizeof(float));
    }

    std::cout << "Read KITTI point cloud with " << points->width << " points, writing to " << out_file << std::endl;
}