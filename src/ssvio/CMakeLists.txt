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

set(GFLAGS_USE_TARGET_NAMESPACE ON)

# find_package(LetNet REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/ssvio/*.cpp src/ui/*.cpp)
file(GLOB_RECURSE headers CONFIGURE_DEPENDS inc/*.h)
add_library(ssvio SHARED
    ${srcs}
)

target_include_directories(ssvio PUBLIC
    inc
)
target_link_libraries(ssvio
    glog::glog
    Sophus::Sophus
    g2o::core g2o::types_sba g2o::types_sim3 g2o::solver_dense g2o::solver_eigen g2o::solver_csparse
    ${Pangolin_LIBRARIES}
    ${DBoW2_LIBRARY}
    ${OpenCV_LIBS}
)

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