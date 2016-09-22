
#thruster node

add_ros_node(thruster_maestro thruster_maestro.cpp)
target_link_libraries(thruster_maestro serial)

add_ros_node(thruster_sabertooth thruster_sabertooth.cpp)
target_link_libraries(thruster_sabertooth serial)

#control system node
add_ros_node(control control.cpp)
