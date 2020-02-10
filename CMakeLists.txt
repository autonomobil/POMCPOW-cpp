cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(ika_decision_making VERSION 0.1 LANGUAGES CXX)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# IF(EXISTS "${PROJECT_SOURCE_DIR}/../../shared_dependencies/boost_1_72_0/lib/")
#   set(Boost_NO_SYSTEM_PATHS=ON)
#   set(BOOST_ROOT ${PROJECT_SOURCE_DIR}/../../shared_dependencies/boost_1_72_0/)
#   message(WARNING "Boost 1.72 found! Using boost version 1.72 from shared_dependencies")
#   find_package(Boost 1.72 REQUIRED EXACT)
# ELSE()
#   message(WARNING "Boost 1.72 NOT found! Using normal boost..")
#   find_package(Boost REQUIRED)
# ENDIF()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  definitions
  rospy
  std_msgs
  lanelet2_utils
  lanelet2_interface_ros
  tf
)

# set(JlCxx_DIR "/home/mo/.julia/packages/CxxWrap/lDNAy/deps/usr/lib/cmake/JlCxx")
# find_package(JlCxx)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   msgSimpleLinefromLanelet.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
  rospy
  message_runtime
  definitions
  lanelet2_utils 
  lanelet2_interface_ros
  tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(ika_decision_making_node SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# set(JlCxx_location "/home/mo/.julia/packages/CxxWrap/lDNAy/deps/usr")
# message(STATUS "Found JlCxx at ${JlCxx_location}")

# # get_target_property(JlCxx_location JlCxx::cxxwrap_julia LOCATION)
# get_filename_component(JlCxx_location ${JlCxx_location} DIRECTORY)
# set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${JlCxx_location}")
# add_library(ika_decision_making SHARED src/main.cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/pomdp.jl
#   # scripts/pomdp_test2.jl
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# target_link_libraries(ika_decision_making JlCxx::cxxwrap_julia)

# install(TARGETS
# ika_decision_making
# LIBRARY DESTINATION lib
# ARCHIVE DESTINATION lib
# RUNTIME DESTINATION lib)

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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

add_executable(
  ika_decision_making_node 
  src/callbacks.cpp
  src/ika_pomdp.cpp 
  src/interpretAction.cpp 
  src/solver.cpp 
  src/tree.cpp 
  src/utils.cpp 
)

target_link_libraries(
  ika_decision_making_node 
  ${catkin_LIBRARIES}
)

target_include_directories(ika_decision_making_node  PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(ika_decision_making_node  ${${ika_decision_making_node}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ika_decision_making.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
