add_rostest_gtest(test_thruster_maestro
  thruster_maestro.test
  test_thruster_maestro.cpp)
add_dependencies(test_thruster_maestro thruster_maestro)
target_link_libraries(test_thruster_maestro serial test_tools ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
