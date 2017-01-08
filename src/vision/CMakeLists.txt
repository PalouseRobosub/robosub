add_ros_node(vision VisionProcessor.cpp Vision.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES})

add_ros_node(camera_repub CameraRepublisher.cpp)

add_ros_node(stereo_vision StereoProcessor.cpp StereoDisparity.cpp)
target_link_libraries(stereo_vision ${OpenCV_LIBRARIES})

add_subdirectory(calib)
