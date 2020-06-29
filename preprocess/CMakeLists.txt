aux_source_directory(. DIR_LIB_PREPROCESS_LIB)
add_library(preprocess ${DIR_LIB_PREPROCESS_LIB} pcd2bin.cpp lidar_in_camera_range.cpp)