cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

project(dataset_reader)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

include_directories(inc)

add_library(dataset_reader src/kitti_dataset.cpp)

# ament_package()