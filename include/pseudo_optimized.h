//
// Created by liuwch on 2020/5/25.
//

#ifndef POINTSCLOUDPROCESS_PSEUDO_OPTIMIZED_H
#define POINTSCLOUDPROCESS_PSEUDO_OPTIMIZED_H

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

void pseudo_optimized(string& lidar_file, string& pseudo_file, string& data_file, string& label_file, string& error_file);

#endif //POINTSCLOUDPROCESS_PSEUDO_OPTIMIZED_H
