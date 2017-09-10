add_rostest_gtest(test_thruster_maestro
  thruster_maestro.test
  test_thruster_maestro.cpp)
add_dependencies(test_thruster_maestro thruster_maestro)
target_link_libraries(test_thruster_maestro serial test_tools ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(test_thruster_sabertooth
  thruster_sabertooth.test
  test_thruster_sabertooth.cpp)
add_dependencies(test_thruster_sabertooth thruster_sabertooth)
target_link_libraries(test_thruster_sabertooth serial test_tools ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest_gtest(test_control_system
  control_system.test
  test_control_system.cpp)
add_dependencies(test_control_system control test_control_keep_alive)
target_link_libraries(test_control_system test_tools ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_ros_node(test_control_keep_alive test_control_keep_alive.cpp)
