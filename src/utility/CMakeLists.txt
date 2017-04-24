add_library(serial serial.cpp)
target_link_libraries(serial ${catkin_LIBRARIES})

add_library(ftdi_serial ftdi_serial.cpp)

add_executable(camera_enumerator camera_enumerator.cpp)
target_link_libraries(camera_enumerator ${catkin_LIBRARIES} libflycapture.so.2)

add_library(test_tools
  test_tools.cpp
  serial_testbench.cpp
  spawn_process.cpp)
target_link_libraries(test_tools ${catkin_LIBRARIES})
