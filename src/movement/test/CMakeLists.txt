add_rostest_gtest(test_sabertooth_thruster
  thruster_sabertooth.test
  test_thruster_sabertooth.cpp)
add_dependencies(test_sabertooth_thruster thruster_sabertooth)
target_link_libraries(test_sabertooth_thruster serial test_tools ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
