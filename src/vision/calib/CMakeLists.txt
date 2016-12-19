add_ros_node(monoCalib Calibration.cpp MonoCalibrator.cpp CalibrationSettings.cpp)
target_link_libraries(monoCalib ${OpenCV_LIBRARIES})

add_ros_node(undistortMono UndistortMono.cpp)
target_link_libraries(undistortMono ${OpenCV_Libraries})
