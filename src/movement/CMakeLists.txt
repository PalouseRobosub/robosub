#thruster nodes
add_ros_node(thruster_maestro thruster_maestro.cpp maestro_thruster_driver.cpp)
target_link_libraries(thruster_maestro serial)

#control system node
add_ros_node(control control.cpp control_system.cpp rotation_engine.cpp)

#add test directory
add_subdirectory(test)
