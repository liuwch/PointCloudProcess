#include <iostream>
#include <map>

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctime>
#include <omp.h>

#include "../include/bin2pcd.h"
#include "../include/pcd2bin.h"
#include "../include/lidar_in_camera_range.h"
#include "../include/pseudo_optimized.h"
#include "../include/pc_vis.h"
#include "../include/testing_datasets.h"
#include "../include/training_datasets.h"


int main() {

    /*-----------------------------------------------------
   * 0. gain dataset index
   * ----------------------------------------------------*/
    vector<string> indices;
    string path_index = "/home/Data1/Datasets/KITTI/object/trainval.txt";
    fstream in_index(path_index.c_str(), ios::in | ios::binary);
    for(int i = 0; in_index.good() & !in_index.eof(); i++) {
        string index;
        in_index >> index;
        indices.push_back(index);
    }
    in_index.close();

    cout << "sample number: " << indices.size() << endl;

    /*-----------------------------------------------------
     * 1. read pcd file
     * ----------------------------------------------------*/
//    vector<string> file_velo_image_view_pcd;
//    vector<string> file_pseudo_velodyne_pcd;
//    vector<string> file_velodyne_bin;
//    vector<string> file_lidar_pcd;
//
//    string velodyne_bin = "/home/Data1/Datasets/KITTI/object/training/pseudo_lidar_velodyne/";
//    string lidar_pcd = "/home/Data1/Datasets/KITTI/object/training/velodyne_pcd/";
//
//    for (int i = 0; i < indices.size(); i++) {
//        file_velo_image_view_pcd.push_back("/home/Data1/Datasets/KITTI/object/training/velo_image_view_pcd/" + indices[i] + ".pcd");
//        file_pseudo_velodyne_pcd.push_back("/home/Data1/Datasets/KITTI/object/training/pseudo_velodyne_pcd/" + indices[i] + ".pcd");
//        file_velodyne_bin.push_back(velodyne_bin + indices[i] + ".bin");
//        file_lidar_pcd.push_back(lidar_pcd + indices[i] + ".pcd");
//    }

    /*-----------------------------------------------------
     * 1. velodyne bin file to pcd file
     * ----------------------------------------------------*/
    string velodyne_pcd = "/home/Data1/Datasets/KITTI/object/training/pcd_file/velodyne_pcd/";
    string velodyne_bin = "/home/Data1/Datasets/KITTI/object/training/velodyne/";
    if (access(velodyne_pcd.c_str(), 0)) {
        cout << velodyne_pcd << " is not existing" << endl;
        mkdir(velodyne_pcd.c_str(), 0777);
    }

    vector<string> file_velodyne_bin;
    vector<string> file_velodyne_pcd;
    for (int i = 0; i < indices.size(); i++) {
        file_velodyne_bin.push_back(velodyne_bin + indices[i] + ".bin");
        file_velodyne_pcd.push_back(velodyne_pcd + indices[i] + ".pcd");
    }

    clock_t start_time, end_time;
    start_time = clock();
    int num_data = indices.size();
    omp_set_num_threads(20);
    // #pragma omp parallel for schedule(static, 500)

    for (int i = 0; i < num_data; i++) {
//        bin2pcd(file_velodyne_bin[i], file_velodyne_pcd[i]);
    }
    end_time = clock();
    // cout << "The run time is: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "The run time is: " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;


    /*-----------------------------------------------------
     * 2. pseudo velodyne bin file to pcd file
     * ----------------------------------------------------*/
    string pseudo_velodyne_bin = "/home/Data1/Datasets/KITTI/object/training/pseudo_lidar_velodyne/";
    string pseudo_velodyne_pcd = "/home/Data1/Datasets/KITTI/object/training/pcd_file/pseudo_velodyne_pcd/";
    if (access(pseudo_velodyne_pcd.c_str(), 0)) {
        cout << pseudo_velodyne_pcd << " is not existing" << endl;
        mkdir(pseudo_velodyne_pcd.c_str(), 0777);
    }
    vector<string> file_pseudo_velodyne_bin;
    vector<string> file_pseudo_velodyne_pcd;
    for (int i = 0; i < indices.size(); i++) {
        file_pseudo_velodyne_bin.push_back(pseudo_velodyne_bin + indices[i] + ".bin");
        file_pseudo_velodyne_pcd.push_back(pseudo_velodyne_pcd + indices[i] + ".pcd");
    }

    omp_set_num_threads(20);
    // #pragma omp parallel for schedule(static, 500)
    for (int i = 0; i < num_data; i++) {
//        bin2pcd(file_pseudo_velodyne_bin[i], file_pseudo_velodyne_pcd[i]);
    }

    /*-----------------------------------------------------
     * 3. generate image view lidar
     * ----------------------------------------------------*/
    string velo_image_view_pcd = "/home/Data1/Datasets/KITTI/object/training/pcd_file/velo_image_view_pcd/";
    if (access(velo_image_view_pcd.c_str(), 0)) {
        cout << velo_image_view_pcd << " is not existing" << endl;
        mkdir(velo_image_view_pcd.c_str(), 0777);
    }
    vector<string> file_velo_image_view_pcd;
    for (int i = 0; i < indices.size(); i++) {
        file_velo_image_view_pcd.push_back(velo_image_view_pcd + indices[i] + ".pcd");
    }

    omp_set_num_threads(24);
//    #pragma omp parallel for schedule(static, 500)
    for (int i = 0; i < num_data; i++) {
//        velo_image_view(file_velodyne_pcd[i], file_velo_image_view_pcd[i], file_pseudo_velodyne_pcd[i]);
    }


    /*-----------------------------------------------------
    * 4. pseudo lidar optimzied
    * ----------------------------------------------------*/
    string pseudo_optimized_pcd = "/home/Data1/Datasets/KITTI/object/training/pcd_file/pseudo_optimized_pcd/";
//    string pseudo_label = "/home/Data1/Datasets/KITTI/object/training/pcd_file/pseudo_label/";
    string pseudo_label = "/home/Data1/Datasets/KITTI/classify/training/label/";
    string error = "/home/Data1/Datasets/KITTI/object/training/pcd_file/error/";
    if (access(pseudo_optimized_pcd.c_str(), 0)) {
        cout << pseudo_optimized_pcd << " is not existing" << endl;
        mkdir(pseudo_optimized_pcd.c_str(), 0777);
    }
    if (access(pseudo_label.c_str(), 0)) {
        cout << pseudo_label << " is not existing" << endl;
        mkdir(pseudo_label.c_str(), 0777);
    }
    if (access(error.c_str(), 0)) {
        cout << error << " is not existing" << endl;
        mkdir(error.c_str(), 0777);
    }
    vector<string> file_pseudo_optimized_pcd;
    vector<string> file_pseudo_label;
    vector<string> file_error;
    for (int i = 0; i < indices.size(); i++) {
        file_pseudo_optimized_pcd.push_back(pseudo_optimized_pcd + indices[i] + ".pcd");
        file_pseudo_label.push_back(pseudo_label + indices[i] + ".bin");
        file_error.push_back(error + indices[i] + ".bin");
    }

//    #pragma omp parallel for schedule(static, 500)
    for (int i = 0; i < indices.size(); i++) {
//        pseudo_optimized(file_velo_image_view_pcd[i], file_pseudo_velodyne_pcd[i], file_pseudo_optimized_pcd[i], file_pseudo_label[i], file_error[i]);
    }


    /*-----------------------------------------------------
    * 5. generate testing datasets
    * ----------------------------------------------------*/
    int i = 1;
    string testing_path = "/home/Data1/Datasets/KITTI/classify/testing/";
    testing_datasets(file_velo_image_view_pcd[i], file_pseudo_velodyne_pcd[i], testing_path, i);

    /*-----------------------------------------------------
    * 5. generate training datasets
    * ----------------------------------------------------*/
    string training_path = "/home/Data1/Datasets/KITTI/classify/training/";
//    training_datasets(file_error[i], file_velo_image_view_pcd[i], file_pseudo_velodyne_pcd[i], training_path, i, 1024);
    hello();



    return 0;
}
