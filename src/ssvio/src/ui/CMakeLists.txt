add_library(ui SHARED
    pangolin_window_impl.cpp
    pangolin_window.cpp
    trajectory_ui.cpp
    cloud_ui.cpp
)

target_include_directories(ui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../inc>
)
target_link_libraries(ui

    glog::glog
    Sophus::Sophus
    g2o::core g2o::types_sba g2o::types_sim3 g2o::solver_dense g2o::solver_eigen
    ${Pangolin_LIBRARIES}
    ${DBoW2_LIBRARY}
    ${OpenCV_LIBS}
)