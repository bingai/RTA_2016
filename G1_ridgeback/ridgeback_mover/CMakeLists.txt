cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_mover)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RotateService.srv
  TranslateService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ridgeback_mover
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/rotate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)