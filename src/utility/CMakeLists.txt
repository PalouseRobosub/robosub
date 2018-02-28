add_library(serial serial.cpp)
target_link_libraries(serial ${catkin_LIBRARIES})

add_library(breathe breathe.cpp)
target_link_libraries(breathe ${catkin_LIBRARIES})

add_library(test_tools
  test_tools.cpp
  serial_testbench.cpp
  spawn_process.cpp)
target_link_libraries(test_tools ${catkin_LIBRARIES})
