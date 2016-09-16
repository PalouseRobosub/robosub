## Declare a C++ executable
 add_executable(pub_sub_example pub_sub.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(pub_sub_example  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(pub_sub_example
   ${catkin_LIBRARIES}
 )
