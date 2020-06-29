//
// Created by liuwch on 2020/5/24.
//

#ifndef POINTSCLOUDPROCESS_PCD2BIN_H
#define POINTSCLOUDPROCESS_PCD2BIN_H

#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

using namespace std;

void pcd2bin(string& in_file, string& out_file);

#endif //POINTSCLOUDPROCESS_PCD2BIN_H
