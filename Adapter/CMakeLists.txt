# MIRA needs cmake >= 2.8
cmake_minimum_required(VERSION 2.8)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# include cmake file for MIRA
include(CMakeLists.mira)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(ros_to_mira src/ros_to_mira.cpp)

# for mira support (auto linking, enabling C++0x support,...)
rosbuild_add_mira(ros_to_mira)
