cmake_minimum_required(VERSION 2.8.3)
project(microwave_detection)
add_definitions("-std=c++0x -Dlinux -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  g1_control
  roscpp
  std_msgs
  world_model_msgs
  tf_conversions

)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8.0 EXACT REQUIRED)
include_directories(.)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

set(SISYPHUS_LIBS sisyphus_graph_utilities sisyphus_registration_utilities sisyphus_sift_engine sisyphus_box_detection sisyphus_pointcloud_utilities sisyphus_io_utilities sisyphus_sift_rgbd_slam)



catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES microwave_detection 
 CATKIN_DEPENDS roscpp std_msgs world_model_msgs g1_control tf_conversions
 DEPENDS PCL ${SISYPHUS_LIBS}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(microwave_detection src/microwave_detect.cpp src/microwave_reconstruct.cpp src/FitSize.cpp  src/FindPlanes.cpp)

target_link_libraries(microwave_detection ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(microwave_detect src/microwave_detect.cpp src/microwave_reconstruct.cpp src/FitSize.cpp src/FindPlanes.cpp)
target_link_libraries(microwave_detect ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

