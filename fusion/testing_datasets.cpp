//
// Created by liuwch on 2020/5/29.
//

# include "../include/testing_datasets.h"

void testing_datasets(string& lidar_file, string& pseudo_file, string& testing_path, int file_index) {
    // training_path = "/home/Data1/Datasets/KITTI/classify/training/"
    // generate 100 pseudo point around pseudo lidar point
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

    cvflann::Index<cvflann::L2_Simple<double> > index(data_pseudo, cvflann::KDTreeSingleIndexParams(15));
    index.buildIndex();

    int knn = 500;
    cvflann::Matrix<int> indices(new int[pseudo_nums * knn], pseudo_nums, knn);
    cvflann::Matrix<double> dists(new double[pseudo_nums * knn], pseudo_nums, knn);

    index.knnSearch(data_pseudo, indices, dists, knn, cvflann::SearchParams());

    vector<string> file_testing_data;
    char s[10];
    sprintf(s, "%06d", file_index);
    string str = s;
    string testing_index_path = testing_path + str;
    cout << testing_index_path << endl;
    if (access(testing_index_path.c_str(), 0)) {
        cout << testing_index_path << " is not existing" << endl;
        mkdir(testing_index_path.c_str(), 0777);
    }


    for (int i = 0; i < pseudo_nums; i++) {
        char s_index[10];
        sprintf(s_index, "%06d", i);
        string str_index = s_index;
        string file_path = testing_index_path + "/" + str_index + ".bin";
        fstream out_file(file_path.c_str(), ios::out | ios::binary);
        for (int k = 0; k < knn; k++) {
            out_file.write((char*)&pseudo->points[indices[i][k]].x, 3 * sizeof(float));
            out_file.write((char*)&pseudo->points[indices[i][k]].intensity, sizeof(float));
        }
        out_file.close();
        cout << file_path << endl;
    }

    delete data_lidar.data;
    delete data_pseudo.data;
    delete indices.data;
    delete dists.data;
}