cmake_minimum_required(VERSION 3.28)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(vslam)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(tracker)