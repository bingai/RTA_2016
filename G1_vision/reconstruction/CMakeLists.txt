cmake_minimum_required(VERSION 2.8.3)
project(reconstruction)

add_definitions("-std=c++0x -Dlinux -O3")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	pcl_conversions
	#pcl_ros
	g1_control
	cv_bridge
	tf_conversions
	reconstruction_msgs
	world_model_msgs
	message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SISYPHUS_LIBS sisyphus_graph_utilities sisyphus_registration_utilities sisyphus_sift_engine sisyphus_box_detection sisyphus_pointcloud_utilities sisyphus_io_utilities sisyphus_sift_rgbd_slam)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES reconstruction reconstruction_utilities
	CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions g1_control tf_conversions reconstruction_msgs world_model_msgs
	DEPENDS OpenCV PCL siftgpu ${SISYPHUS_LIBS}
)

add_library(reconstruction_utilities src/reconstruction_utilities.cpp)
target_link_libraries(reconstruction_utilities siftgpu ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_library(reconstruction src/reconstruction.cpp)
target_link_libraries(reconstruction reconstruction_utilities siftgpu ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(reconstruction_test src/reconstruction_test.cpp)
target_link_libraries(reconstruction_test reconstruction siftgpu ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(reconstruction_from_file src/reconstruction_from_file.cpp)
target_link_libraries(reconstruction_from_file reconstruction_utilities siftgpu ${SISYPHUS_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
