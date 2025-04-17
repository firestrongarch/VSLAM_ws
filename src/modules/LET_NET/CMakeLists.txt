cmake_minimum_required(VERSION 3.28)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CXX_STANDARD 20)

project(LetNet)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ncnn REQUIRED)

# file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp src/*.cc)
file(GLOB_RECURSE headers CONFIGURE_DEPENDS inc/*.h)

# Build
add_library(${PROJECT_NAME} SHARED src/letnet.cpp ${headers})
target_include_directories(${PROJECT_NAME}  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc>
    INTERFACE
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBS}
    ncnn
)

add_executable(test test/test.cpp)
target_link_libraries(test PUBLIC
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
)

# install
install(TARGETS ${PROJECT_NAME} 
    EXPORT ${PROJECT_NAME}Targets)
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# # Install cmake targets
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install header files
install(FILES ${headers}     
    DESTINATION include/${PROJECT_NAME}
)