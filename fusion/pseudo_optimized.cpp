//
// Created by liuwch on 2020/5/25.
//
#include "../include/pseudo_optimized.h"

//bool cmp(pair<int, double> a, pair<int, double> b) {
//    return a.second < b.second;
//}

// return index based on error in ascending order
void pseudo_optimized(string& lidar_file, string& pseudo_file, string& data_file, string& pseudo_label_file, string& error_file) {
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

    // 将符合要求的点写入到data_file文件中去
    vector<int> data_optimized;
    vector<int> label;
    for (int i = 0; i < pseudo_nums; i++) {
        if (dists[i][0] < 0.0001 && abs(lidar->points[indices[i][0]].x - pseudo->points[i].x) < 1) {
            data_optimized.push_back(i);
            label.push_back(1);
        }
        else {
            label.push_back(0);
        }
    }
    cout << pseudo_label_file << endl;

    // write label to label_file
    ofstream label_file(pseudo_label_file.c_str(), ios::out | ios::binary);
    for (int i = 0; i < label.size(); i++) {
        label_file.write((char*)&label[i], sizeof(int));
    }
    label_file.close();


    // create map, {index, dists}
    map<int, double> mp;
    int count = 0;
    for (int i = 0; i < pseudo_nums; i++) {
        double error = abs(lidar->points[indices[i][0]].x - pseudo->points[i].x);
        mp[i] = error;
        if (label[i] == 1) {
            count++;
        }
    }

    // write error to file
    ofstream error(error_file.c_str(), ios::out | ios::binary);
    for (int i = 0; i < mp.size(); i++) {
        error.write((char*)&mp[i], sizeof(double));
    }
    error.close();

    // test

//    for (int i = 0; i < 100; i++) {
//        cout << dists[i][0] << " " << dists[i][9] <<   " " << mp[i] << " " << label[i]<< endl;
//    }
//    cout << "the nu  of 1: " << count << endl;
//    cout << pseudo_nums << endl;
//    cout << label.size() << endl;
//    cout << lidar_nums << endl;
//    cout << data_optimized.size() << endl;

    // map sort
//    vector<pair<int, double>> vec;
//    for (map<int, double>::iterator it = mp.begin(); it != mp.end(); it++) {
//        vec.push_back(pair<int, double>(it->first, it->second));
//    }
//    sort(vec.begin(), vec.end(), cmp);
//

    // 将优化后的点放入新建的点云中去
    pcl::PointCloud<pcl::PointXYZI>::Ptr pseudo_optimized(new pcl::PointCloud<pcl::PointXYZI>);
    pseudo_optimized->width = data_optimized.size();
    pseudo_optimized->height = 1;
    pseudo_optimized->is_dense = false;
    pseudo_optimized->points.resize(pseudo_optimized->width * pseudo_optimized->height);

    for (int k = 0; k < data_optimized.size(); k++) {
        pseudo_optimized->points[k] = pseudo->points[data_optimized[k]];
    }

    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZI>(data_file, *pseudo_optimized, false);


    std::cout << "Read KITTI point cloud with " << pseudo_optimized->width << " points, writing to " << data_file << std::endl;


    delete data_lidar.data;
    delete data_pseudo.data;
//    delete indices_pseudo.data;
//    delete dists_pseudo.data;
    delete indices.data;
    delete dists.data;

}