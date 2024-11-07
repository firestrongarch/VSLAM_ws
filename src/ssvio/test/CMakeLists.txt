
add_executable(test_ssvio test_ssvio.cpp)
target_link_libraries(test_ssvio ui ssvio)

add_executable(ssvio_uma ssvio_uma.cpp)
target_link_libraries(ssvio_uma ui ssvio LetNet::LetNet)
