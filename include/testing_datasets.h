//
// Created by liuwch on 2020/5/29.
//

#ifndef POINTSCLOUDPROCESS_TRAINING_DATASETS_H
#define POINTSCLOUDPROCESS_TRAINING_DATASETS_H

#include "opencv2//flann/flann.hpp"
#include "opencv2//flann/miniflann.hpp"
#include "opencv2//flann/kdtree_index.h"
#include "opencv2//flann/matrix.h"
#include "opencv2//flann/miniflann.hpp"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <map>
#include <vector>
#include <fstream>
#include <string>
#include <algorithm>

using namespace cv;
using namespace std;

void testing_datasets(string& lidar_file, string& pseudo_file, string& testing_path, int file_index);

#endif //POINTSCLOUDPROCESS_TRAINING_DATASETS_H
