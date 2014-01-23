project(photo_node)

include_directories(include ../../include)

add_executable(photo_node photo_node.cpp)
target_link_libraries(photo_node 
    photo2 
    ${catkin_LIBRARIES} 
    ${GPHOTO2_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)
