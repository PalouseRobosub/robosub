
#example publisher/subscriber node
add_ros_node(pub_sub_example pub_sub.cpp)
add_ros_node(fork_parent fork_parent.cpp)
add_ros_node(fork_child fork_child.cpp)
add_ros_node(param_example parameter.cpp)

add_ros_node(serial_talker serial_talker.cpp)
target_link_libraries(serial_talker serial)
