cmake_minimum_required(VERSION 3.20)
project(feature_tracker)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

find_package(LetNet REQUIRED)

include_directories("../camera_model/include")

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
)

ament_target_dependencies(feature_tracker rclcpp rcpputils std_msgs sensor_msgs geometry_msgs cv_bridge camera_model image_transport)
target_link_libraries(feature_tracker ${OpenCV_LIBS} LetNet::LetNet)

add_executable(feature_tracker_illustration
    src/feature_tracker_illu.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
)
ament_target_dependencies(feature_tracker_illustration rclcpp rcpputils std_msgs sensor_msgs geometry_msgs cv_bridge camera_model image_transport)
target_link_libraries(feature_tracker_illustration ${OpenCV_LIBS} LetNet::LetNet)

# Install nodes
install(
  TARGETS feature_tracker feature_tracker_illustration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()