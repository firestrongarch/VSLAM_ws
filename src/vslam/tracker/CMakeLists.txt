file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp inc/*.h)
add_library(tracker STATIC ${srcs})
target_include_directories(tracker PUBLIC inc ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tracker PUBLIC ${OpenCV_LIBS} Sophus::Sophus)