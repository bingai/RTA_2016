cmake_minimum_required(VERSION 2.8.3)
project(object_choice_server)
add_definitions("-std=c++0x -Dlinux -O3")


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  world_model_msgs
  graph_filter
)

find_package(PCL 1.8 EXACT REQUIRED)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(object_choice src/object_choice.cpp)
target_link_libraries(object_choice ${catkin_LIBRARIES} ${PCL_LIBRARIES})

