
# if(CATKIN_ENABLE_TESTING)
# find_package(rostest REQUIRED)
# add_rostest_gtest(test_serial_subscriber
#  serial_subscriber.test
#  test_serial_subscriber.cpp)
# target_link_libraries(test_serial_subscriber serial_testbench serial ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
# endif()

add_rostest_gtest(test_pub_sub
 pub_sub.test
 test_pub_sub.cpp)
target_link_libraries(test_pub_sub ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
