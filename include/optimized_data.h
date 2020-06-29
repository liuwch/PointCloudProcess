//
// Created by liuwch on 2020/5/26.
//

#ifndef POINTSCLOUDPROCESS_OPTIMIZED_DATA_H
#define POINTSCLOUDPROCESS_OPTIMIZED_DATA_H

#include "opencv2//flann/flann.hpp"
#include "opencv2//flann/miniflann.hpp"
#include "opencv2//flann/kdtree_index.h"
#include "opencv2//flann/matrix.h"
#include "opencv2//flann/miniflann.hpp"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <iostream>
#include <map>

using namespace cv;
using namespace std;

void optimized_data(string &lidar_file, string &pseudo_file, string &pseudo_optimized_file, double T, double D);

#endif //POINTSCLOUDPROCESS_OPTIMIZED_DATA_H
