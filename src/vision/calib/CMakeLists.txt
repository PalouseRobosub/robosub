add_ros_node(monoCalib Calibration.cpp MonoCalibrator.cpp CalibrationSettings.cpp)
target_link_libraries(monoCalib ${OpenCV_LIBRARIES})

add_ros_node(stereo_calib StereoCalibration.cpp StereoCalibrator.cpp CalibrationSettings.cpp)
target_link_libraries(stereo_calib ${OpenCV_LIBRARIES})

add_ros_node(undistortion Undistortion.cpp)
target_link_libraries(undistortion ${OpenCV_Libraries})
