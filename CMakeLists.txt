cmake_minimum_required(VERSION 2.8.3)

project(photo)
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    roslib 
    sensor_msgs 
    cv_bridge 
    message_runtime 
    message_generation
)

find_package(OpenCV REQUIRED)

add_service_files(
    FILES
    Capture.srv
    GetConfig.srv
    SetConfig.srv
    )
generate_messages(
    DEPENDENCIES
    sensor_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs
    DEPENDS libgphoto2
    )


add_subdirectory(src)

