add_library(localization_system localization_system.cpp)
add_dependencies(localization_system  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(particle_filter particle_filter.cpp)
add_dependencies(particle_filter  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(lin_accel_kalman_filter lin_accel_kalman_filter.cpp)
add_dependencies(lin_accel_kalman_filter  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(filter_sensors filter_sensors.cpp)
add_dependencies(filter_sensors  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_ros_node(localization localization.cpp)
target_link_libraries(localization localization_system particle_filter lin_accel_kalman_filter filter_sensors)
