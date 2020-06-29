//
// Created by liuwch on 2020/5/24.
//

#include "../include/bin2pcd.h"

void bin2pcd(string &in_file, string &out_file) {

	// load point cloud
	std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary); // 以二进制方式输入
	if (!input.good()) {  // 读写有任何问题，false
		std::cerr << "Could not read file: " << in_file << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);  // 流指针被改变为指向从文件开始计算的一个绝对位置。

	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

	int i = 0;
//    char ch = input.peek();
//    while(ch != EOF) {
//        pcl::PointXYZI point;
//        input.read((char*)&point.x, 3 * sizeof(float));
//        input.read((char*)&point.intensity, sizeof(float));
//        points->push_back(point);
//        ch = input.peek();
//        i++;
//    }

    for (i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char*)&point.x, 3 * sizeof(float));
        input.read((char*)&point.intensity, sizeof(float));
        if (input.fail())
            break;
        points->push_back(point);
    }

	input.close();

	std::cout << "Read KITTI point cloud with " << i << " points, writing to " << out_file << std::endl;
	pcl::PCDWriter writer;
	writer.write< pcl::PointXYZI >(out_file, *points, false);
}