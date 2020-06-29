//
// Created by liuwch on 2020/5/29.
//

#include "../include/training_datasets.h"

bool cmp(pair<int, double> a, pair<int, double> b) {
    return a.second < b.second;
}

void training_datasets(string& error_file, string& lidar_file, string& pseudo_file, string& training_path, int file_index, int sample_num) {
    fstream error(error_file.c_str(), ios::in | ios::binary);
    map<int, double> mp;
    int i = 0;
    char ch = error.peek();
    while(ch != EOF) {
        error.read((char*)&mp[i], sizeof(double));
        ch = error.peek();
        i++;
    }
    error.close();
    cout << "Training data num: " << i << endl;

    vector<pair<int, double> > vec;
    for (map<int, double>::iterator it = mp.begin(); it != mp.end(); it++) {
        vec.push_back(pair<int, double>(it->first, it->second));
    }

    sort(vec.begin(), vec.end(), cmp);

//    sel_index: save the index of 1000 min error and 1000 max error in the pseudo point order
    vector<int> sel_index;
    for (int i = 0; i < sample_num; i++) {
        sel_index.push_back(vec[i].first);
    }
    for (int i = vec.size() - sample_num; i < vec.size(); i++) {
        sel_index.push_back(vec[i].first);
    }

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
    cout << sel_index.size() << endl;
//    for (int i = 0; i < sel_index.size(); i++) {
//        cout << i << " " << sel_index[i] << " " << pseudo->points[indices[sel_index[i]][0]] << " " << pseudo->points[indices[sel_index[i]][1]] << endl;
//    }

    // training data file
    vector<string> file_training_data;
    char s[10];
    sprintf(s, "%06d", file_index);
    string str = s;
    string training_index_path = training_path + str;
    cout << training_index_path << endl;
    if (access(training_index_path.c_str(), 0)) {
        cout << training_index_path << " is not existing" << endl;
        mkdir(training_index_path.c_str(), 0777);
    }

    for (int i = 0; i < sel_index.size(); i++) {
        char s_index[10];
        sprintf(s_index, "%06d", i);
        string str_index = s_index;
        string file_path = training_index_path + "/" + str_index + ".bin";
        fstream out_file(file_path.c_str(), ios::out | ios::binary);
        for (int k = 0; k < knn; k++) {
            out_file.write((char*)&pseudo->points[indices[ sel_index[i] ][k]].x, 3 * sizeof(float));
            out_file.write((char*)&pseudo->points[indices[ sel_index[i] ][k]].intensity, sizeof(float));
        }
        out_file.close();

//        cout << file_path << endl;
    }

    // write label to file
    string label_path = training_path + "label" + "/" + str + ".bin";
    cout << label_path << endl;
    fstream label_file(label_path.c_str(), ios::out | ios::binary);
    for (int i = 0; i < sel_index.size(); i++) {
        if (i < sample_num) {
            int label = 1;
            label_file.write((char*)&label, sizeof(int));
        }
        else {
            int label = 0;
            label_file.write((char*)&label, sizeof(int));
        }
    }
    label_file.close();


    delete data_lidar.data;
    delete data_pseudo.data;
    delete indices.data;
    delete dists.data;
}

void hello() {
    cout << "liuwch" << endl;
}