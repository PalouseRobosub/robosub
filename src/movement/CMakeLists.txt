
#thruster node
add_ros_node(thruster thruster.cpp)
target_link_libraries(thruster serial)

#control system node
add_ros_node(control control.cpp)
