cmake_minimum_required(VERSION 2.8.3)
project(world_model_projections)

add_definitions("-std=c++0x -Dlinux -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  world_model
  world_model_msgs
  eigen_conversions
)

find_package(OpenCV REQUIRED)
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


add_executable(world_projection src/world_projection.cpp)
target_link_libraries(world_projection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

