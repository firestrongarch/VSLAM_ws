cmake_minimum_required(VERSION 4.1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-pthread -fopenmp=libomp)
set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD d0edc3af-4c50-42ea-a356-e2862fe7a444)
set(CMAKE_CXX_MODULE_STD 1)
set(CMAKE_CXX_EXTENSIONS ON)

project(Yslam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_PREFIX_PATH "E:/CPP/vcpkg/installed/x64-mingw-dynamic")
set(CMAKE_BUILD_PARALLEL_LEVEL 1)

find_package(tomlplusplus CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ncnn REQUIRED)
find_package(Boost REQUIRED COMPONENTS lockfree)

file(GLOB_RECURSE 3rd_sources CONFIGURE_DEPENDS cppm/3rd/*.cppm)
add_library(3rd SHARED)
target_sources(3rd PUBLIC FILE_SET CXX_MODULES FILES ${3rd_sources})
target_link_libraries(3rd PUBLIC ${OpenCV_LIBS} tomlplusplus::tomlplusplus ncnn Boost::lockfree)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS cppm/yslam/*.cppm)
add_library(Yslam SHARED)
target_sources(Yslam PUBLIC FILE_SET CXX_MODULES FILES ${srcs})
target_link_libraries(Yslam PUBLIC 3rd)

add_executable(slam main.cpp)
target_link_libraries(slam PRIVATE Yslam)
add_executable(test test/test.cpp)
target_link_libraries(test PRIVATE Yslam)
