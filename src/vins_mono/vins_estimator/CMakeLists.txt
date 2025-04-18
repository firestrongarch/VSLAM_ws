
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV 4 REQUIRED)

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(vins_estimator
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    )

ament_target_dependencies(vins_estimator rclcpp rcpputils std_msgs sensor_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge image_transport)

target_link_libraries(vins_estimator ${OpenCV_LIBS} ${CERES_LIBRARIES} camera_model) 
target_include_directories(vins_estimator PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../camera_model/include")

# Install nodes
install(
  TARGETS vins_estimator
  DESTINATION lib/${PROJECT_NAME}
)
