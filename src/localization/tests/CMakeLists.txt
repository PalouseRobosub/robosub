add_rostest_gtest(test_localization_system
    localizationSystem.test
    test_localization_system.cpp)
#add_dependencies(test_localization_system)
target_link_libraries(test_localization_system test_tools ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

