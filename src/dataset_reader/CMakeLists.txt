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

# 修改：在安装库时增加 EXPORT 选项
install(TARGETS dataset_reader
  EXPORT dataset_readerTargets
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY inc/dataset_reader
  DESTINATION include
)

ament_export_targets(dataset_readerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rcpputils Poco)
ament_package()

include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/dataset_readerConfig.cmake"
    INSTALL_DESTINATION lib/cmake/dataset_reader
)

install(EXPORT dataset_readerTargets
    FILE dataset_readerTargets.cmake
    NAMESPACE dataset_reader::
    DESTINATION lib/cmake/dataset_reader
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/dataset_readerConfig.cmake"
    DESTINATION lib/cmake/dataset_reader
)

