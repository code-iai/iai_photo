project(test_photo)

include_directories(include ../../include)
add_executable(test_photo_config test_photo_config.cpp)
add_executable(test_photo_capture test_photo_capture.cpp)
target_link_libraries(test_photo_config photo2 ${catkin_LIBRARIES})
target_link_libraries(test_photo_capture photo2 ${catkin_LIBRARIES})
