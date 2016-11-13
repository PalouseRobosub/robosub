add_rostest_gtest(test_maestro_thruster
  maestro_launch.test
  test_maestro_thruster.cpp)
add_dependencies(test_maestro_thruster thruster_maestro)
target_link_libraries(test_maestro_thruster serial test_tools ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
