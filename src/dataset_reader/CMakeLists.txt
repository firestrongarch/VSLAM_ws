cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

project(dataset_reader)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

find_package(Poco REQUIRED COMPONENTS Foundation)

include_directories(inc)

add_library(dataset_reader src/kitti_dataset.cpp)
target_link_libraries(dataset_reader Poco::Foundation)

add_executable(test_kitti test/test_kitti.cpp)
target_link_libraries(test_kitti dataset_reader)
# ament_package()