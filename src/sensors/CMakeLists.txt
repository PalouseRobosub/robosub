add_library(bno055 Bno055.cpp)
target_link_libraries(bno055 ${catkin_LIBRARIES})

add_ros_node(bno055_sensor bno055_sensor.cpp)
target_link_libraries(bno055_sensor serial bno055)

add_ros_node(imu imu.cpp)
target_link_libraries(imu serial)

add_ros_node(depth depth.cpp)
target_link_libraries(depth serial)

add_ros_node(hydrophones trilateration.cpp)

add_ros_node(print_orientation print_orientation.cpp)

add_library(localization_system localization_system.cpp)
add_dependencies(localization_system  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_ros_node(localization localization.cpp)
target_link_libraries(localization localization_system)
