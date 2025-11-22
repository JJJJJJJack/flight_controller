cmake_minimum_required(VERSION 2.8.3)
project(flight_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  mav_msgs
  mav_planning_msgs)


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

catkin_install_python(PROGRAMS
scripts/communication.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Declare a cpp executable
add_executable(bicopter_optitrack_controller 
  src/bicopter_optitrack_controller.cpp
  )

add_executable(zyz_controller 
  src/GNC2022/zyz_controller.cpp
)

add_executable(lyf_controller 
  src/GNC2022/lyf_controller.cpp
  )

add_executable(lwj_controller 
  src/GNC2022/lwj_controller.cpp
)

add_executable(yjr_controller 
  src/GNC2022/yjr_controller.cpp
)

add_executable(cs_controller 
  src/cs_controller.cpp
)

add_executable(autoflip_controller 
  src/autoflip_controller.cpp
)

add_executable(pusherBi_controller 
  src/pusherBi_controller.cpp
)

add_executable(pullerBi_controller 
  src/pullerBi_controller.cpp
)

add_executable(autoflip_controller_for_quad
  src/autoflip_controller_for_quad.cpp
)

add_executable(autoflip_controller_forbidFlip
  src/autoflip_controller_forbidFlip.cpp
)
add_executable(tailsitter_controller
  src/tailsitter_controller.cpp
)
add_executable(tailsitter_controller_new
  src/tailsitter_controller_new.cpp
)
add_executable(bode_attitude_test
  src/bode_attitude_test.cpp
)

add_executable(autoflip_controller_cascade 
  src/autoflip_controller_cascade.cpp
)

add_executable(flycontroller_minsnap 
  src/flycontroller_minsnap.cpp
)
add_executable(quadtilt_controller
src/quadtilt_controller.cpp
)
add_executable(bicopter_through
src/bicopter_through.cpp
)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(flight_controller_node flight_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bicopter_optitrack_controller
  ${catkin_LIBRARIES}
  )

target_link_libraries(zyz_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(lyf_controller
  ${catkin_LIBRARIES}
  )
  
target_link_libraries(tailsitter_controller
  ${catkin_LIBRARIES}
  )

  target_link_libraries(tailsitter_controller_new
  ${catkin_LIBRARIES}
  )
  
target_link_libraries(lwj_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(yjr_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(cs_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(autoflip_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(pusherBi_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(pullerBi_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(autoflip_controller_for_quad
  ${catkin_LIBRARIES}
)

target_link_libraries(autoflip_controller_forbidFlip
  ${catkin_LIBRARIES}
)
target_link_libraries(bode_attitude_test
  ${catkin_LIBRARIES}
)

target_link_libraries(autoflip_controller_cascade
  ${catkin_LIBRARIES}
)

target_link_libraries(flycontroller_minsnap
  ${catkin_LIBRARIES}
)

target_link_libraries(quadtilt_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(bicopter_through
  ${catkin_LIBRARIES}
)
