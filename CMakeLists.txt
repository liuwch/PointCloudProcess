cmake_minimum_required(VERSION 3.16)
project(PointsCloudProcess)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories("${PROJECT_SOURCE_DIR}/include")

add_subdirectory(preprocess)
add_subdirectory(fusion)
add_subdirectory(visualization)

# aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
# add_executable(PointsCloudProcess ${DIR_SRCS} include/pcd2bin.h)
add_executable(PointsCloudProcess src/main.cpp)

target_link_libraries(PointsCloudProcess ${PCL_LIBRARIES})
target_link_libraries(PointsCloudProcess ${OpenCV_LIBS} )
target_link_libraries(PointsCloudProcess preprocess)
target_link_libraries(PointsCloudProcess fusion)
target_link_libraries(PointsCloudProcess visualization)

set(CMAKE_BUILD_TYPE "Release")