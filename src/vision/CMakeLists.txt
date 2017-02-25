add_ros_node(vision VisionProcessor.cpp StereoProcessor.cpp Vision.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES})

add_ros_node(camera_repub CameraRepublisher.cpp)

add_subdirectory(calib)
