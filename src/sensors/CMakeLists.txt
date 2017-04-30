add_library(bno055 Bno055.cpp)
target_link_libraries(bno055 ${catkin_LIBRARIES})

add_library(trax PniTrax.cpp)
target_link_libraries(trax ${catkin_LIBRARIES})

add_ros_node(bno055_sensor bno055_sensor.cpp)
target_link_libraries(bno055_sensor serial bno055)

add_ros_node(bno055_calibrate bno055_calibration.cpp)
target_link_libraries(bno055_calibrate serial bno055)

add_ros_node(imu imu.cpp)
target_link_libraries(imu serial)

add_ros_node(depth depth.cpp)
target_link_libraries(depth serial)

add_ros_node(trax_sensor trax_sensor.cpp)
target_link_libraries(trax_sensor serial trax)

add_ros_node(hydrophones trilateration.cpp)
