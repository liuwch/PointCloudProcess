# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/Data1/liuwch/Downloads/clion-2020.1.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/Data1/liuwch/Downloads/clion-2020.1.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/Data1/liuwch/CLionProjects/PointsCloudProcess

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug

# Include any dependencies generated for this target.
include preprocess/CMakeFiles/preprocess.dir/depend.make

# Include the progress variables for this target.
include preprocess/CMakeFiles/preprocess.dir/progress.make

# Include the compile flags for this target's objects.
include preprocess/CMakeFiles/preprocess.dir/flags.make

preprocess/CMakeFiles/preprocess.dir/bin2pcd.cpp.o: preprocess/CMakeFiles/preprocess.dir/flags.make
preprocess/CMakeFiles/preprocess.dir/bin2pcd.cpp.o: ../preprocess/bin2pcd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object preprocess/CMakeFiles/preprocess.dir/bin2pcd.cpp.o"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/preprocess.dir/bin2pcd.cpp.o -c /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/bin2pcd.cpp

preprocess/CMakeFiles/preprocess.dir/bin2pcd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/preprocess.dir/bin2pcd.cpp.i"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/bin2pcd.cpp > CMakeFiles/preprocess.dir/bin2pcd.cpp.i

preprocess/CMakeFiles/preprocess.dir/bin2pcd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/preprocess.dir/bin2pcd.cpp.s"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/bin2pcd.cpp -o CMakeFiles/preprocess.dir/bin2pcd.cpp.s

preprocess/CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.o: preprocess/CMakeFiles/preprocess.dir/flags.make
preprocess/CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.o: ../preprocess/lidar_in_camera_range.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object preprocess/CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.o"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.o -c /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/lidar_in_camera_range.cpp

preprocess/CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.i"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/lidar_in_camera_range.cpp > CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.i

preprocess/CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.s"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/lidar_in_camera_range.cpp -o CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.s

preprocess/CMakeFiles/preprocess.dir/pcd2bin.cpp.o: preprocess/CMakeFiles/preprocess.dir/flags.make
preprocess/CMakeFiles/preprocess.dir/pcd2bin.cpp.o: ../preprocess/pcd2bin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object preprocess/CMakeFiles/preprocess.dir/pcd2bin.cpp.o"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/preprocess.dir/pcd2bin.cpp.o -c /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/pcd2bin.cpp

preprocess/CMakeFiles/preprocess.dir/pcd2bin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/preprocess.dir/pcd2bin.cpp.i"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/pcd2bin.cpp > CMakeFiles/preprocess.dir/pcd2bin.cpp.i

preprocess/CMakeFiles/preprocess.dir/pcd2bin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/preprocess.dir/pcd2bin.cpp.s"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess/pcd2bin.cpp -o CMakeFiles/preprocess.dir/pcd2bin.cpp.s

# Object files for target preprocess
preprocess_OBJECTS = \
"CMakeFiles/preprocess.dir/bin2pcd.cpp.o" \
"CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.o" \
"CMakeFiles/preprocess.dir/pcd2bin.cpp.o"

# External object files for target preprocess
preprocess_EXTERNAL_OBJECTS =

preprocess/libpreprocess.a: preprocess/CMakeFiles/preprocess.dir/bin2pcd.cpp.o
preprocess/libpreprocess.a: preprocess/CMakeFiles/preprocess.dir/lidar_in_camera_range.cpp.o
preprocess/libpreprocess.a: preprocess/CMakeFiles/preprocess.dir/pcd2bin.cpp.o
preprocess/libpreprocess.a: preprocess/CMakeFiles/preprocess.dir/build.make
preprocess/libpreprocess.a: preprocess/CMakeFiles/preprocess.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libpreprocess.a"
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && $(CMAKE_COMMAND) -P CMakeFiles/preprocess.dir/cmake_clean_target.cmake
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/preprocess.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
preprocess/CMakeFiles/preprocess.dir/build: preprocess/libpreprocess.a

.PHONY : preprocess/CMakeFiles/preprocess.dir/build

preprocess/CMakeFiles/preprocess.dir/clean:
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess && $(CMAKE_COMMAND) -P CMakeFiles/preprocess.dir/cmake_clean.cmake
.PHONY : preprocess/CMakeFiles/preprocess.dir/clean

preprocess/CMakeFiles/preprocess.dir/depend:
	cd /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/Data1/liuwch/CLionProjects/PointsCloudProcess /home/Data1/liuwch/CLionProjects/PointsCloudProcess/preprocess /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess /home/Data1/liuwch/CLionProjects/PointsCloudProcess/cmake-build-debug/preprocess/CMakeFiles/preprocess.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : preprocess/CMakeFiles/preprocess.dir/depend

