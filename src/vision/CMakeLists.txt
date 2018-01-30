add_ros_node(undistortion fisheye_undistortion.cpp)
target_link_libraries(undistortion ${OpenCV_LIBRARIES})
