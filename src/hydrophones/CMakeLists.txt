add_library(data_stream DataStream.cpp DataStreamServer.cpp AnalogPacket.cpp)
add_dependencies(data_stream ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_ros_node(hydrophone_stream hydrophone_stream_bin.cpp)
target_link_libraries(hydrophone_stream data_stream ${catkin_LIBRARIES})
