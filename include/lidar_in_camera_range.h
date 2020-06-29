//
// Created by liuwch on 2020/5/25.
//

#ifndef POINTSCLOUDPROCESS_LIDAR_IN_CAMERA_RANGE_H
#define POINTSCLOUDPROCESS_LIDAR_IN_CAMERA_RANGE_H

#include <pcl/point_cloud.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cmath>
#include <vector>

#define pi 3.1415926
using namespace std;

double* get_camera_range(string &in_file);
void velo_image_view(string& in_file, string& out_file, string& view_angle_file);

#endif //POINTSCLOUDPROCESS_LIDAR_IN_CAMERA_RANGE_H
