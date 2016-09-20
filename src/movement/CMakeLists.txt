
#thruster node
add_ros_node(thruster_maistro thruster_maistro.cpp)
target_link_libraries(thruster_maistro serial)

#control system node
add_ros_node(control control.cpp)
