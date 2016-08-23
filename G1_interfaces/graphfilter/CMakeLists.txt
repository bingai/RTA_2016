cmake_minimum_required(VERSION 2.8.3)
project(graph_filter)
add_definitions("-std=c++0x -Dlinux -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf_conversions
)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8.0 EXACT REQUIRED)
include_directories(.)

add_definitions("-std=c++0x -Dlinux -O3")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

#set(SISYPHUS_LIBS sisyphus_graph_utilities sisyphus_transform_utilities sisyphus_sift_engine sisyphus_box_detection sisyphus_pointcloud_utilities sisyphus_misc_utilities sisyphus_siftrgbdslam)

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES graph_filter
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


add_library(graph_filter src/graph_test.cpp src/GraphFilter.cpp src/FitSize.cpp  src/FindPlanes.cpp src/ZAdaptiveNormals.cc src/Constraints.cpp src/NodeConstraints.cpp
 src/EdgeConstraints.cpp src/SizeConstraints.cpp src/LocationConstraints.cpp 
 src/FingerConstraints.cpp src/ArmFingerConstraints.cpp src/Centroid.cpp src/Pca.cpp
 src/GraphUtilities.cpp src/Trainer.cpp)

target_link_libraries(graph_filter ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(graph_test src/graph_test.cpp src/GraphFilter.cpp src/FitSize.cpp src/FindPlanes.cpp src/ZAdaptiveNormals.cc src/Constraints.cpp src/NodeConstraints.cpp
  src/EdgeConstraints.cpp src/SizeConstraints.cpp src/LocationConstraints.cpp 
  src/FingerConstraints.cpp src/ArmFingerConstraints.cpp src/Centroid.cpp src/Pca.cpp
  src/GraphUtilities.cpp src/Trainer.cpp)
target_link_libraries(graph_test ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

