add_subdirectory(filters)

add_ros_node(vision VisionProcessor.cpp StereoProcessor.cpp FeatureProcessor.cpp Vision.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES} vision_filters)

add_ros_node(camera_repub CameraRepublisher.cpp)

add_subdirectory(calib)
