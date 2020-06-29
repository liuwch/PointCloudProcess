//
// Created by liuwch on 2020/5/29.
//


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

bool cmp(pair<int, double> a, pair<int, double> b);
void training_datasets(string& error_file, string& lidar_file, string& pseudo_file, string& training_path, int file_index, int sample_num);
void hello();


