project(photo2)
include_directories(include ../../include)

add_library(photo2 photo_reporter.cpp photo_camera_list.cpp photo_camera.cpp photo_image.cpp)

#pkg_check_modules(GPHOTO2 REQUIRED libgphoto2)
#include_directories(${GPHOTO2_INCLUDE_DIRS})
#link_directories(${GPHOTO2_LIBRARY_DIRS})


target_link_libraries(photo2 ${LIBGPHOTO2_LIBRARIES} ${OpenCV_LIBRARIES})
