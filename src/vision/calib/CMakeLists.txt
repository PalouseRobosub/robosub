add_ros_node(monoCalib Calibration.cpp MonoCalibrator.cpp)
target_link_libraries(monoCalib ${OpenCV_LIBRARIES})
