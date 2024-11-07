add_library(ssvio SHARED
    system.cpp
    frontend.cpp
    camera.cpp
    orbextractor.cpp
    frame.cpp
    feature.cpp
    keyframe.cpp
    map.cpp
    mappoint.cpp
    setting.cpp
    orbpattern.cpp
    backend.cpp
    loopclosing.cpp
)

target_include_directories(ssvio PUBLIC
    ${DBoW2_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../inc>
)
target_link_libraries(ssvio 
    glog::glog
    Sophus::Sophus
    g2o::core g2o::types_sba g2o::types_sim3 g2o::solver_dense g2o::solver_eigen g2o::solver_csparse
    ${Pangolin_LIBRARIES}
    ${DBoW2_LIBRARY}
    ${OpenCV_LIBS}
)