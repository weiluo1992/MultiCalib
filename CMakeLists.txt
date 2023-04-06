# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)


project ("MultiCalib")

# 包含子项目。

add_subdirectory ("CameraIntrinsicCalibration")
add_subdirectory ("Camera2LidarDemo")
add_subdirectory ("JointCalibration")
