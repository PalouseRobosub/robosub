add_library(serial serial.cpp)
target_link_libraries(serial ${catkin_LIBRARIES})

add_library(ftdi_serial ftdi_serial.cpp)

# add_library(spawn_process spawn_process.cpp)
# target_link_libraries(spawn_process ${catkin_LIBRARIES})
#
# add_library(serial_testbench serial_testbench.cpp)
# target_link_libraries(serial_testbench spawn_process ${catkin_LIBRARIES})

add_library(test_tools
  test_tools.cpp
  serial_testbench.cpp
  spawn_process.cpp)
target_link_libraries(test_tools ${catkin_LIBRARIES})
