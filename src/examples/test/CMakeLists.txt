
add_rostest_gtest(test_serial_subscriber
  serial_subscriber.test
  test_serial_subscriber.cpp)
add_dependencies(test_serial_subscriber serial_subscriber_example)
target_link_libraries(test_serial_subscriber serial test_tools ${catkin_LIBRARIES} ${GTEST_LIBRARIES})



add_rostest_gtest(test_pub_sub
  pub_sub.test
  test_pub_sub.cpp)
add_dependencies(test_pub_sub pub_sub_example)
target_link_libraries(test_pub_sub ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
