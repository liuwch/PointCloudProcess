//
// Created by liuwch on 2020/5/26.
//

#include "../include/optimized_data.h"

// keep useful point to optimized point cloud
void optimized_data(string &lidar_file, string &pseudo_file, string &pseudo_optimized_file, string &label_file, double T, double D) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pseudo(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZI>(lidar_file, *lidar);
    reader.read<pcl::PointXYZI>(pseudo_file, *pseudo);

    int pseudo_nums = pseudo->size();
    cvflann::Matrix<double> data_pseudo(new double[pseudo_nums * 2], pseudo_nums, 2);
    for (int i = 0; i < pseudo_nums; i++) {
        data_pseudo[i][0] = (double)pseudo->points[i].y;
        data_pseudo[i][1] = (double)pseudo->points[i].z;
    }

    int lidar_nums = lidar->size();
    cvflann::Matrix<double> data_lidar(new double[lidar_nums * 2], lidar_nums, 2);
    for (int i = 0; i < lidar_nums; i++) {
        data_lidar[i][0] = lidar->points[i].y;
        data_lidar[i][1] = lidar->points[i].z;
    }

    cvflann::Index<cvflann::L2_Simple<double> > index(data_lidar, cvflann::KDTreeSingleIndexParams(15));
    index.buildIndex();

    int knn = 10;
    cvflann::Matrix<int> indices(new int[pseudo_nums * knn], pseudo_nums, knn);
    cvflann::Matrix<double> dists(new double[pseudo_nums * knn], pseudo_nums, knn);

    index.knnSearch(data_pseudo, indices, dists, knn, cvflann::SearchParams());


    // 将误差较大的点排除， 保存的点 包括 认为合理的点 和 无法确认是否合理的点
    vector<int> points_bad;
    for (int i = 0; i < pseudo_nums; i++) {
        points_bad.push_back(i);
        if (dists[i][0] < D) {
            if (abs(lidar->points[indices[i][0]].x - pseudo->points[i].x) > T) {
                points_bad.pop_back();
            }
        }
    }

    // 将误差较小的点保存下来
    vector<int> points_used;
    for (int i = 0; i < pseudo_nums; i++) {
        if (dists[i][0] < D) {
            if (abs(lidar->points[indices[i][0]].x - pseudo->points[i].x) < T) {
                points_used.push_back(i);
            }
        }
    }

    cout << points_bad.size() << " " << points_used.size() << " " << pseudo->size() << endl;

    // 将优化后的点放入新建的点云中去
    pcl::PointCloud<pcl::PointXYZI>::Ptr pseudo_optimized(new pcl::PointCloud<pcl::PointXYZI>);
    pseudo_optimized->width = points_used.size();
    pseudo_optimized->height = 1;
    pseudo_optimized->is_dense = false;
    pseudo_optimized->points.resize(pseudo_optimized->width * pseudo_optimized->height);

    for (int k = 0; k < points_used.size(); k++) {
        pseudo_optimized->points[k] = pseudo->points[points_used[k]];
    }

    pcl::PCDWriter writer;
    std::cout << "Read KTTI point cloud with " << pseudo_optimized->width << " points, writing to " << pseudo_optimized_file << std::endl;
    writer.write<pcl::PointXYZI>(pseudo_optimized_file, *pseudo_optimized, false);

    // 释放内存
    delete data_lidar.data;
    delete data_pseudo.data;
    delete indices.data;
    delete dists.data;
}