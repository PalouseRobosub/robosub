add_library(trax PniTrax.cpp)
target_link_libraries(trax ${catkin_LIBRARIES})

add_ros_node(imu imu.cpp)

add_ros_node(depth depth.cpp)
target_link_libraries(depth serial)

add_ros_node(trax_sensor trax_sensor.cpp)
target_link_libraries(trax_sensor serial trax)

add_ros_node(trax_calibrate trax_calibration.cpp)
target_link_libraries(trax_calibrate serial trax)

add_ros_node(pinger_bearing pinger_bearing.cpp)
