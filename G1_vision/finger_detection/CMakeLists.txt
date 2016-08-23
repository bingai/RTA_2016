cmake_minimum_required(VERSION 2.8.3)
project(finger_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 EXACT REQUIRED)

catkin_package(
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(finger_detection src/finger_detection.cpp)
target_link_libraries(finger_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
