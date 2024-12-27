cmake_minimum_required(VERSION 3.28)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

project(vins_mono)

find_package(ament_cmake REQUIRED)

add_subdirectory(camera_model)
add_subdirectory(feature_tracker)
add_subdirectory(vins_estimator)
add_subdirectory(pose_graph)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()