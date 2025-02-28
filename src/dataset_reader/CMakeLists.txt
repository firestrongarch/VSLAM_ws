cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

project(dataset_reader)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

find_package(Poco REQUIRED COMPONENTS Foundation)

add_library(dataset_reader src/kitti_dataset.cpp)
target_link_libraries(dataset_reader PUBLIC Poco::Foundation)
target_include_directories(dataset_reader PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

add_executable(test_kitti test/test_kitti.cpp)
target_link_libraries(test_kitti dataset_reader)


include(CMakePackageConfigHelpers) # 包含配置帮助模块

install(DIRECTORY inc/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# 生成配置文件
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)


