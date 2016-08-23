cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_launcher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
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

