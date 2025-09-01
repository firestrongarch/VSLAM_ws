cmake_minimum_required(VERSION 4.1.0)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

add_compile_options(-pthread -fopenmp=libomp)

set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD d0edc3af-4c50-42ea-a356-e2862fe7a444)
set(CMAKE_CXX_MODULE_STD 1)
set(CMAKE_CXX_EXTENSIONS ON)
project(Yslam VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_PREFIX_PATH "E:/CPP/vcpkg/installed/x64-mingw-dynamic")
find_package(tomlplusplus CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ncnn REQUIRED)

add_library(cv SHARED)
target_sources(cv PUBLIC FILE_SET CXX_MODULES FILES src/cv.cppm)
target_link_libraries(cv PUBLIC ${OpenCV_LIBS})

add_executable(slam slam.cpp)
target_link_libraries(slam PRIVATE cv)