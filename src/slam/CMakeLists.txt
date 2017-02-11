add_library(obstacle_map obstacle_map.cpp)
add_dependencies(obstacle_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_ros_node(slam slam.cpp)
target_link_libraries(slam obstacle_map)
