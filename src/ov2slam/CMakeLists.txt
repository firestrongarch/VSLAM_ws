cmake_minimum_required(VERSION 3.28)

set(CMAKE_CXX_STANDARD 20)
project(ov2slam)

find_package(OpenCV REQUIRED)
set(WITH_OPENCV_CONTRIB OFF)
if (OpenCV_FOUND)
  message(STATUS "Found OpenCV lib : ${OpenCV_VERSION} - ${OpenCV_INCLUDE_DIRS}")
  if(WITH_OPENCV_CONTRIB)
    message(STATUS "Going to use OpenCV contrib!")
    add_definitions(-DOPENCV_CONTRIB)
  endif()
endif (OpenCV_FOUND)

find_package(Eigen3 REQUIRED)
find_package(opengv)
if (opengv_FOUND)
  message(STATUS "Found Opengv lib : ${opengv_INCLUDE_DIR} - ${opengv_DIR}")
  add_definitions(-DUSE_OPENGV)
endif (opengv_FOUND)

find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)

##########
# iBOW-LCD
##########
find_package(obindex2 REQUIRED)
find_package(ibow_lcd REQUIRED)

set(WITH_IBOW_LCD OFF)
if (ibow_lcd_FOUND)
  set(WITH_IBOW_LCD ON)
  add_definitions(-DIBOW_LCD)
  message(STATUS "iBoW-LCD found!  Going to use Loop Closer! : ${lcdetector_VERSION} - ${lcdetector_DIR}")
endif (ibow_lcd_FOUND)

find_package(jsoncpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(backward_ros REQUIRED)

##################################################

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE headers CONFIGURE_DEPENDS inc/*.h)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED ${srcs} ${headers})

# Add mandatory dependencies
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/ceres_parametrization/
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  Sophus::Sophus
  Ceres::ceres
  ${OpenCV_LIBRARIES}
)

if (ibow_lcd_FOUND)
  target_link_libraries(
    ${PROJECT_NAME}
      ibow_lcd::ibow_lcd
  )
endif (ibow_lcd_FOUND)

if (opengv_FOUND)
  target_link_libraries(
    ${PROJECT_NAME}
      opengv
  )
endif (opengv_FOUND)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
  pcl_conversions
  PCL
  std_msgs
  cv_bridge
  image_transport
  # backward_ros
)

install(DIRECTORY include/ DESTINATION include/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ov2slam_node.cpp)

add_executable(${PROJECT_NAME}_node test/ov2slam_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  image_transport
  std_msgs
  sensor_msgs
  cv_bridge
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_datasets test/ov2slam_datasets.cpp)

ament_target_dependencies(${PROJECT_NAME}_datasets
  rclcpp
  image_transport
  std_msgs
  sensor_msgs
  cv_bridge
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_datasets
  ${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()