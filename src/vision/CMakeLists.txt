add_subdirectory(filters)
add_subdirectory(feature_detectors)

add_ros_node(vision ColorProcessor.cpp StereoProcessor.cpp FeatureProcessor.cpp Vision.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES} vision_filters feature_detectors elas)

add_ros_node(camera_repub CameraRepublisher.cpp)

add_subdirectory(calib)
