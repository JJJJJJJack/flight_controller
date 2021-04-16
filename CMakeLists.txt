cmake_minimum_required(VERSION 2.8.3)
project(flight_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  tf
  message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Signal.msg
#   Message2.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
  # Or other packages containing msgs
#)

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
#  INCLUDE_DIRS include
#  LIBRARIES serial_to_sbus
  CATKIN_DEPENDS #other_catkin_pkg
  message_runtime  std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  utility
#  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
# add_library(flight_controller
#   src/${PROJECT_NAME}/flight_controller.cpp
# )

## Declare a cpp executable
add_executable(bicopter_optitrack_controller 
  src/bicopter_optitrack_controller.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(flight_controller_node flight_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bicopter_optitrack_controller
  ${catkin_LIBRARIES}
)
