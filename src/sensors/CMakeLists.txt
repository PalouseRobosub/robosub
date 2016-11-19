

add_ros_node(imu imu.cpp)
target_link_libraries(imu serial)


add_ros_node(depth depth.cpp)
target_link_libraries(depth serial)

add_ros_node(print_orientation print_orientation.cpp)
add_ros_node(simulator_bridge simulator_bridge.cpp)
