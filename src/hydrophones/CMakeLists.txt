add_library(data_stream DataStream.cpp DataStreamServer.cpp AnalogPacket.cpp MetadataPacket.cpp)
add_dependencies(data_stream ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_ros_node(hydrophone_stream hydrophone_stream_bin.cpp)
target_link_libraries(hydrophone_stream data_stream ${catkin_LIBRARIES})

add_ros_node(hydrophone_logger stream_logger.cpp)
target_link_libraries(hydrophone_logger ${catkin_LIBRARIES})

add_ros_node(hydrophone_correlator cross_correlator_bin.cpp)
target_link_libraries(hydrophone_correlator ${catkin_LIBRARIES})

add_ros_node(scrappy scrappy_publisher.cpp)
target_link_libraries(scrappy ${catkin_LIBRARIES})
