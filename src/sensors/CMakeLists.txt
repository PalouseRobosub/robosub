add_ros_node(bno055_sensor bno055_sensor.cpp Bno055.cpp)
target_link_libraries(bno055_sensor serial)

add_ros_node(imu imu.cpp)
target_link_libraries(imu serial)

add_ros_node(bno bno_example.cpp Bno055.cpp)
target_link_libraries(bno serial)

add_ros_node(depth depth.cpp)
target_link_libraries(depth serial)

add_ros_node(print_orientation print_orientation.cpp)
