//
// Created by liuwch on 2020/5/25.
//
#include "../include/lidar_in_camera_range.h"

double* get_camera_range(string &in_file) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZI>(in_file, *cloud);

    vector<double> angle_h;
    vector<double> angle_v;

    for (size_t i = 0; i < cloud->points.size(); i++) {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
        double z = cloud->points[i].z;

        angle_h.push_back(atan(y / x));
        angle_v.push_back(atan(z / x));
    }
    vector<double>::iterator biggest_h = max_element(begin(angle_h), end(angle_h));
    vector<double>::iterator smallest_h = min_element(begin(angle_h), end(angle_h));

    vector<double>::iterator biggest_v = max_element(begin(angle_v), end(angle_v));
    vector<double>::iterator smallest_v = min_element(begin(angle_v), end(angle_v));

    // cout << *biggest << " " << *smallest << endl;
    static double range[4] = { *smallest_h, *biggest_h, *smallest_v, *biggest_v };
    static double range_v[2] = { *smallest_v, *biggest_v };
    // cout << range[0] << " " << range[1] << endl;
    return range;
}

// 将LiDAR数据投影到camera范围内
void velo_image_view(string& in_file, string& out_file, string& view_angle_file) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZI>(in_file, *cloud);

    int points_num = cloud->points.size();
    double h_fov[] = { -45, 45 };
    double v_fov[] = { -24.9, 2.0 };
    double* range = get_camera_range(view_angle_file);

    vector<int> index;  // 保存在image可视范围内的点的index

    for (int i = 0; i < points_num; i++) {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
        double z = cloud->points[i].z;

        if ((atan(y / x) < range[1]) && (atan(y / x) > range[0]) && x > 0) {
            index.push_back(i);
        }
    }

    // 将符合条件的点写入一个新的点云文件

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_filter->width = index.size();
    cloud_filter->height = 1;
    cloud_filter->is_dense = false;
    cloud_filter->points.resize(cloud_filter->width * cloud_filter->height);

    for (int i = 0; i < index.size(); i++) {
        cloud_filter->points[i] = cloud->points[index[i]];
    }

    std::cout << "Read KITTI point cloud with " << cloud_filter->width << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>(out_file, *cloud_filter, false);
}