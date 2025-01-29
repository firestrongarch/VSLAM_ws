cmake_minimum_required(VERSION 3.5)
project(config_pkg)

# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

# 检查文件是否存在
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/voc/brief_k10L6.bin")
    message("File exists: brief_k10L6.bin")
else()
    file(DOWNLOAD 
        https://gh.llkk.cc/https://github.com/dongbo19/VINS-MONO-ROS2/raw/refs/heads/main/config_pkg/support_files/brief_k10L6.bin
        ${CMAKE_CURRENT_LIST_DIR}/voc/brief_k10L6.bin
        TIMEOUT 60
        STATUS  download_vins
        LOG download_log
    )

    # 检查下载状态
    list(GET download_vins 0 status_code)
    list(GET download_vins 1 status_string)
    if(status_code EQUAL 0)
        message("Download succeeded: ${status_string}")
    else()
        # message(FATAL_ERROR "Download failed: ${status_string}\nLog: ${download_log}")
        file(REMOVE ${CMAKE_CURRENT_LIST_DIR}/voc/brief_k10L6.bin)
        message(FATAL_ERROR "Download failed")
    endif()
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/voc/ORBvoc.txt.tar.gz")
    message("File exists: ORBvoc.txt.tar.gz")
else()
    file(DOWNLOAD 
        https://gh.llkk.cc/https://github.com/raulmur/ORB_SLAM2/raw/refs/heads/master/Vocabulary/ORBvoc.txt.tar.gz
        ${CMAKE_CURRENT_LIST_DIR}/voc/ORBvoc.txt.tar.gz
        TIMEOUT 60
        STATUS  download_orb
        LOG download_log
    )

    # 检查下载状态
    list(GET download_orb 0 status_code)
    list(GET download_orb 1 status_string)
    if(status_code EQUAL 0)
        message("Download succeeded: ${status_string}")
    else()
        file(REMOVE ${CMAKE_CURRENT_LIST_DIR}/voc/ORBvoc.txt.tar.gz)
        message(FATAL_ERROR "Download failed: ${status_string}\nLog: ${download_log}")
        message(FATAL_ERROR "Download failed")   
    endif()
endif()

install(DIRECTORY
    config
    voc
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()