cmake_minimum_required(VERSION 3.28)
project(ssvio)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)

# set(CMAKE_CXX_FLAGS "-w")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -ggdb ${CMAKE_CXX_FLAGS} -lpthread")
# set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(LetNet REQUIRED)

find_package(glog REQUIRED)

add_subdirectory(src/ui)
add_subdirectory(src/ssvio)
add_subdirectory(test)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
    TARGETS test_ssvio

    # LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    # ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)