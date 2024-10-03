cmake_minimum_required(VERSION 3.29.0)
project(test)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

add_executable(test main.cpp)
target_link_libraries(test Sophus::Sophus)