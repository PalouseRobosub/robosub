
add_library(bno055 Bno055.cpp)
target_link_libraries(bno055 ${catkin_LIBRARIES})


add_ros_node(imu imu.cpp)
target_link_libraries(imu serial)

add_ros_node(bno bno_example.cpp)
target_link_libraries(bno serial bno055)

add_ros_node(depth depth.cpp)
target_link_libraries(depth serial)

add_ros_node(print_orientation print_orientation.cpp)
