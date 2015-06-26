cmake_minimum_required(VERSION 2.8.3)
project(sparseRRT)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hector_quadrotor_teleop
  roscpp
  rospy
  tf
  std_msgs
  message_generation
  ##ros_lib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   sparse_search.srv
   rrt_search.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
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
  INCLUDE_DIRS include
#  LIBRARIES sparseRRT
  CATKIN_DEPENDS hector_quadrotor roscpp rospy tf std_msgs actionlib_msgs actionlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(sparseRRT
#   src/${PROJECT_NAME}/sparseRRT.cpp
# )


add_library(sparserrtlib
            src/image_creation/svg_image.cpp 
	    #src/input/sst_quadrotor.cpp
            src/motion_planners/planner.cpp
            src/motion_planners/rrt.cpp
            src/motion_planners/sst.cpp
            src/nearest_neighbors/graph_nearest_neighbors.cpp
            src/systems/quadrotor.cpp
            src/utilities/condition_check.cpp
            src/utilities/eulerquat.cpp
            src/utilities/parameter_reader.cpp
            src/utilities/random.cpp
            src/utilities/timer.cpp)
#add_dependencies(quadrotor_sst quad_gaz_generate_messages_cpp)


add_executable(quadrotor_sst_search src/quadrotor_sst_search.cpp)
target_link_libraries(quadrotor_sst_search sparserrtlib
  ${catkin_LIBRARIES}
)

add_executable(quadrotor_rrt_search src/quadrotor_rrt_search.cpp)
target_link_libraries(quadrotor_rrt_search sparserrtlib
  ${catkin_LIBRARIES}
)

add_executable(quadrotor_sst src/quadrotor_sst.cpp)
target_link_libraries(quadrotor_sst sparserrtlib
  ${catkin_LIBRARIES}
)

add_executable(quadrotor_rrt src/quadrotor_rrt.cpp)
target_link_libraries(quadrotor_rrt sparserrtlib
  ${catkin_LIBRARIES}
)



add_executable(display_obstacles src/display_obstacles.cpp)
#add_dependencies(display_obstacles quad_gazebo_generate_messages_cpp)
target_link_libraries(display_obstacles
  ${catkin_LIBRARIES}
)

add_executable(display_trajectory src/display_trajectory.cpp)
#add_dependencies(display_obstacles quad_gazebo_generate_messages_cpp)
target_link_libraries(display_trajectory
  ${catkin_LIBRARIES}
)



## Declare a cpp executable
# add_executable(sparseRRT_node src/sparseRRT_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sparseRRT_node sparseRRT_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(sparseRRT_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS sparseRRT sparseRRT_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sparseRRT.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
