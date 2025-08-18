cmake_minimum_required(VERSION 3.28)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CXX_STANDARD 23)

project(ORB_SLAM2)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(DBoW2 REQUIRED)

# find_package(realsense2)
# find_package(LetNet REQUIRED)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp src/*.cc)
file(GLOB_RECURSE headers CONFIGURE_DEPENDS inc/*.h)

add_library(${PROJECT_NAME} SHARED ${srcs} ${headers})
target_include_directories(${PROJECT_NAME} PUBLIC

    # ${DBoW2_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Sophus::Sophus
    g2o::core g2o::types_sba g2o::types_sim3 g2o::solver_dense g2o::solver_eigen
    ${Pangolin_LIBRARIES}
    ${DBoW2_LIBRARY}
    ${OpenCV_LIBS}
)

# Build examples
add_executable(orb2_stereo_kitti
    Examples/Stereo/stereo_kitti.cc)
target_link_libraries(orb2_stereo_kitti ${PROJECT_NAME})

add_executable(orb2_stereo_uma
    Examples/Stereo/stereo_uma.cc)
target_link_libraries(orb2_stereo_uma ${PROJECT_NAME}

    # LetNet::LetNet
)

add_executable(orb2_rgbd
    Examples/rgbd_tum.cc)
target_link_libraries(orb2_rgbd ${PROJECT_NAME})

add_executable(test
    Examples/test.cc)
target_link_libraries(test ${PROJECT_NAME})