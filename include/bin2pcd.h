//
// Created by liuwch on 2020/5/24.
//

#ifndef POINTSCLOUDPROCESS_BIN2PCD_H
#define POINTSCLOUDPROCESS_BIN2PCD_H

#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

void bin2pcd(string &in_file, string &out_file);

#endif //POINTSCLOUDPROCESS_BIN2PCD_H
