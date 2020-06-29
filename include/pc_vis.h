//
// Created by liuwch on 2020/5/25.
//

#ifndef POINTSCLOUDPROCESS_PC_VIS_H
#define POINTSCLOUDPROCESS_PC_VIS_H

#include<pcl/visualization/cloud_viewer.h>
#include<iostream>                             // 标准C++库中的输入输出类相关头文件。
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>                     // pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>                   // PCL中支持的点类型头文件。
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

using namespace std;

void pc_vis(string &in_file);

#endif //POINTSCLOUDPROCESS_PC_VIS_H
