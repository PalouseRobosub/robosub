
#example publisher/subscriber node
add_ros_node(pub_sub_example pub_sub.cpp)
add_ros_node(fork_parent_example fork_parent.cpp)
add_ros_node(fork_child_example fork_child.cpp)
add_ros_node(param_example parameter.cpp)
add_ros_node(data_analyzer_example data_analyzer.cpp)

add_dynamic_ros_node(dynamicParameters_example DynamicParameters.cpp)

add_ros_node(bno bno.cpp)
target_link_libraries(bno serial bno055)

add_ros_node(serial_subscriber_example serial_subscriber.cpp)
target_link_libraries(serial_subscriber_example serial)

add_ros_node(serial_publisher_example serial_publisher.cpp)
target_link_libraries(serial_publisher_example serial)


#add test directory
add_subdirectory(test)
