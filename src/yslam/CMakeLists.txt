cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Yslam VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_PREFIX_PATH "E:/CPP/vcpkg/installed/x64-mingw-dynamic")
find_package(tomlplusplus CONFIG REQUIRED)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp src/*.cc)
add_library(Yslam SHARED
    ${srcs}
)
target_include_directories(Yslam PUBLIC inc)
target_link_libraries(Yslam PRIVATE tomlplusplus::tomlplusplus)

add_executable(slam main.cpp)
target_link_libraries(slam PRIVATE Yslam)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
