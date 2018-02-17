cmake_minimum_required(VERSION 2.8.3)
project(robosub)

## By default no optimization or debug flags are set use the following
## two lines to pass in these flags. Only use one at a time!
#set(CMAKE_BUILD_TYPE Debug) #Uncomment this line to enable debug symbols
#set(CMAKE_BUILD_TYPE Release) #Uncomment this for max optimization

#enable C++11 support
set(CMAKE_CXX_FLAGS "-Wall -Werror -std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if ("$ENV{ROS_DISTRO}" STRGREATER "indigo")
  #Running Kinetic
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    dynamic_reconfigure
    std_msgs
    message_generation
    roslint
    rostest
    message_filters
    tf
    image_transport
    cv_bridge
    pointgrey_camera_driver
    robosub_msgs
  )

else ()
  #Running Indigo
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    dynamic_reconfigure
    std_msgs
    message_generation
    roslint
    rostest
    message_filters
    tf
    image_transport
    cv3_bridge
    pointgrey_camera_driver
    robosub_msgs
  )
endif()

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Setup ROSlint with our preferred settings
## C++ Using the custom linter to avoid the ROS overloads:

# Recursively select c++ source files
file (GLOB_RECURSE FILES *.cpp *.h *.hpp) #Set the files to parse

# Run the custom linter
# To change the options for the CPP linter edit the CPPLINT.cfg file int the
#     src directory
roslint_custom(${CMAKE_CURRENT_LIST_DIR}/tools/linters/cpplint "--extensions=cpp,hpp,h" ${FILES})

## Python
# To change the options for the python linter edit the PYLINT.cfg file in the
#     tools/linters directory
set(ROSLINT_PYTHON_OPTS "--config=tools/linters/PYLINT.cfg")
roslint_python("./")

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  pneumatic_status.srv
)

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#  AI.action
#)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
# )

# Build platformio message headers
add_custom_target(
        firmware
        COMMAND rm -rf ${CMAKE_CURRENT_LIST_DIR}/firmware/ros_lib
        COMMAND rosrun rosserial_client make_libraries ${CMAKE_CURRENT_LIST_DIR}/firmware/
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Dynamic.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS src
 LIBRARIES test_tools serial
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure std_msgs message_runtime
  DEPENDS OpenCV
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robosub.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#macro for adding and linking nodes to the ros framework
macro(add_ros_node node_name file_name)
  add_executable(${node_name} ${file_name} ${ARGN})
  add_dependencies(${node_name}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node_name} ${catkin_LIBRARIES})
endmacro(add_ros_node)

#macro for adding and linking nodes with dynamic parameters
macro(add_dynamic_ros_node node_name file_name)
  add_executable(${node_name} ${file_name})
    #${PROJECT_NAME}_gencfg is needed for any nodes needing dynamic params
  add_dependencies(${node_name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node_name} ${catkin_LIBRARIES})
endmacro(add_dynamic_ros_node)

#specify where other CMakeLists.txt files live
add_subdirectory(src/examples)
add_subdirectory(src/utility)
add_subdirectory(src/movement)
add_subdirectory(src/sensors)
add_subdirectory(src/controllers)
add_subdirectory(src/vision)
add_subdirectory(src/localization)
