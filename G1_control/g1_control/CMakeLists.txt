cmake_minimum_required(VERSION 2.8.3)
project(g1_control)

find_package(Boost REQUIRED COMPONENTS
  date_time
  system
)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  cmake_modules
  g1_control_msgs
  hand_controller
  tf
  eigen_conversions
  actionlib
  actionlib_msgs
  grasp_generator
)

set(THIS_PACKAGE_INCLUDE_DIRS
  executor_interface/include
  planning_scene_interface/include
)

catkin_package(
  LIBRARIES
    g1_control_planning_scene_interface
    g1_control_executor_interface
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    g1_control_msgs
    hand_controller
    tf
    eigen_conversions
    actionlib
    actionlib_msgs
    grasp_generator
  DEPENDS
    boost
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_subdirectory(planning_scene_interface)
add_subdirectory(executor_interface)

