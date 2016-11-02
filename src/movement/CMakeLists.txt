
#thruster node

add_ros_node(thruster_maestro thruster_maestro.cpp maestro_thruster_controller.cpp)
target_link_libraries(thruster_maestro serial)

add_ros_node(thruster_sabertooth thruster_sabertooth.cpp)
target_link_libraries(thruster_sabertooth serial)

add_library(control_system control_system.cpp)
add_library(rotation_engine rotation_engine.cpp)

#control system node
add_ros_node(control control.cpp)
target_link_libraries(control control_system)
target_link_libraries(control rotation_engine)
