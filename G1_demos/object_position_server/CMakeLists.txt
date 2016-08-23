cmake_minimum_required(VERSION 2.8.3)
project(object_position_server)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  world_model_msgs
)

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


add_executable(update_position src/update_position.cpp)
target_link_libraries(update_position ${catkin_LIBRARIES} ${PCL_LIBRARIES})

