cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(object_detection_3d)
set (CMAKE_BUILD_TYPE Release)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../bin")
# set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../lib")
# set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../lib")
# set (CMAKE_MODULE_PATH "/home/aecins/Dropbox/Research/Code/CMakeModules")
# add_definitions("-Wall -pedantic")

### ----------------------------------------------------------------------------
### Namaris
### ----------------------------------------------------------------------------

# find_package (Namaris REQUIRED)
# message (STATUS "")
# if (Namaris_FOUND)
#   message (STATUS " ---------------------------")
#   message (STATUS " Found Namaris version ${Namaris_VERSION}")
#   message (STATUS "     * include dirs: ${NAMARIS_INCLUDE_DIRS}")
# else()
#   message (FATAL_ERROR : "Namaris NOT FOUND!")
# endif()

# include_directories (${Namaris_INCLUDE_DIRS})

# include_directories ("/home/aecins/Dropbox/Research/Code/namaris/include")

### ----------------------------------------------------------------------------
### PCL
### ----------------------------------------------------------------------------

find_package(PCL 1.8.0 REQUIRED)
message (STATUS "")
if (PCL_FOUND)
  message (STATUS " -----------------------")
  message (STATUS " Found PCL version ${PCL_VERSION}")
  message (STATUS "     * include dirs: ${PCL_INCLUDE_DIRS}")
  message (STATUS "     * library dirs: ${PCL_LIBRARY_DIRS}")
  message (STATUS "     * libraries:    ${PCL_LIBRARIES}")
else()
  message (FATAL_ERROR : "PCL NOT FOUND!")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### ----------------------------------------------------------------------------
### Boost
### ----------------------------------------------------------------------------

find_package(Boost REQUIRED COMPONENTS regex) 
message (STATUS "")
if(Boost_FOUND)
  message (STATUS " --------------------------")
  message (STATUS " Found Boost version ${Boost_VERSION}")
  message (STATUS "     * include dirs: ${Boost_INCLUDE_DIRS}")
  message (STATUS "     * libraries:    ${Boost_LIBRARIES}")
else()
  message (FATAL_ERROR : "Boost NOT FOUND!")    
endif()

### ----------------------------------------------------------------------------
### OpenCV
### ----------------------------------------------------------------------------

find_package(OpenCV REQUIRED)

### ----------------------------------------------------------------------------
### Project dependencies
### ----------------------------------------------------------------------------

set (includes ./include)
include_directories (${includes})
set(SISYPHUS_LIBS sisyphus_graph_utilities sisyphus_registration_utilities sisyphus_sift_engine sisyphus_box_detection sisyphus_box_fitting sisyphus_pointcloud_utilities sisyphus_io_utilities sisyphus_sift_rgbd_slam)

### ----------------------------------------------------------------------------
### C++11 support
### ----------------------------------------------------------------------------

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

### ----------------------------------------------------------------------------
### Libraries
### ----------------------------------------------------------------------------

add_library (object_detection_3d  src/table_segmentation.cpp
                                  src/object_segmentation.cpp
                                  src/rotational_object_detection.cpp
                                  src/handle_detection.cpp
                                  src/box_fitting.cpp
                                  src/fridge_inner_segmentation.cpp
                                  src/projection_utilities.cpp
                                  )
target_link_libraries ( object_detection_3d ${SISYPHUS_LIBS} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

### ----------------------------------------------------------------------------
### Executables
### ----------------------------------------------------------------------------

# Full pipeline
#add_executable (object_detection_3d_test src/object_detection_3d_test.cpp)
#target_link_libraries (object_detection_3d_test ${SISYPHUS_LIBS} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} object_detection_3d)

add_executable (object_detection_3d_tabletop_test src/object_detection_3d_tabletop_test.cpp)
target_link_libraries (object_detection_3d_tabletop_test ${SISYPHUS_LIBS} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} object_detection_3d)

add_executable (object_detection_3d_fridge_test src/object_detection_3d_fridge_test.cpp)
target_link_libraries (object_detection_3d_fridge_test ${SISYPHUS_LIBS} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} object_detection_3d)

# # Rotational segmentation only
# add_executable (object_detection_3d_segmentation_test src/object_detection_3d_segmentation_test.cpp)
# target_link_libraries (object_detection_3d_segmentation_test ${PCL_LIBRARIES} ${Boost_LIBRARIES} object_detection_3d)
