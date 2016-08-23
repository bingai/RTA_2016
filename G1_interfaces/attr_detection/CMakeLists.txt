cmake_minimum_required(VERSION 2.8.3)
project(attr_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf_conversions
  graph_filter
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.8.0 EXACT REQUIRED)
include_directories(.)

add_definitions("-std=c++0x -Dlinux -O3")


catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES attr_detection
 CATKIN_DEPENDS roscpp std_msgs tf_conversions
 DEPENDS PCL
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(attr_detection src/active_detect.cpp src/AttributeProcess.cpp)
target_link_libraries(attr_detection ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(attr_detect src/active_detect.cpp src/AttributeProcess.cpp)
target_link_libraries(attr_detect ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

