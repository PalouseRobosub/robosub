
#thruster node

add_ros_node(thruster_maestro thruster_maestro.cpp)
target_link_libraries(thruster_maestro serial)

add_ros_node(thruster_sabertooth thruster_sabertooth.cpp)
target_link_libraries(thruster_sabertooth serial)

add_library(control_system control_system.cpp)
add_dependencies(control_system  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_library(rotation_engine rotation_engine.cpp)

#control system node
add_ros_node(control control.cpp)
target_link_libraries(control control_system)
target_link_libraries(control rotation_engine)
