cmake_minimum_required(VERSION 3.29.0)
project(test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Sophus REQUIRED PATHS /home/fu/VSLAM_ws/install/Sophus/share NO_DEFAULT_PATH)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(test main.cpp)