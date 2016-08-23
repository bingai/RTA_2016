cmake_minimum_required(VERSION 2.8)

project(openni_calibration)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../bin")

### ----------------------------------------------------------------------------
### Openmp
### ----------------------------------------------------------------------------

find_package(OpenMP REQUIRED)
message (STATUS "")
if (OPENMP_FOUND)
  message (STATUS "")
  message (STATUS " ------------")
  message (STATUS " Found OpenMP")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
  message (FATAL_ERROR : "OpenMP NOT FOUND!")  
endif()
message (STATUS "")

### ----------------------------------------------------------------------------
### Add libraries
### ----------------------------------------------------------------------------

add_library(${CALIBRATION_LIB_TARGET} SHARED openni_calibration.cpp)
target_link_libraries (openni_calibration ${OpenCV_LIBS} ${PCL_LIBRARIES})

### ----------------------------------------------------------------------------
### Add executables
### ----------------------------------------------------------------------------

# Calibrate intrinsics
add_executable (openni_intrinsic_calibration ../../applications/calibration/intrinsic_calibration.cpp )
target_link_libraries (openni_intrinsic_calibration ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CALIBRATION_LIB_TARGET})

# Calibrate extrinsics
add_executable (openni_extrinsic_calibration ../../applications/calibration/extrinsic_calibration.cpp )
target_link_libraries (openni_extrinsic_calibration ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CALIBRATION_LIB_TARGET})

# Calibrate depth and IR offset
add_executable (openni_depth_ir_calibration ../../applications/calibration/depth_ir_calibration.cpp )
target_link_libraries (openni_depth_ir_calibration ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${CALIBRATION_LIB_TARGET})

# Calibrate depth
add_executable (openni_depth_calibration ../../applications/calibration/depth_calibration.cpp )
target_link_libraries (openni_depth_calibration ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${CALIBRATION_LIB_TARGET})

# Check calibration results
add_executable (openni_calibration_check ../../applications/calibration/calibration_check.cpp )
target_link_libraries (openni_calibration_check ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CALIBRATION_LIB_TARGET})

#-------------------------- INSTALLATION --------------------------------------
install(TARGETS
  ${CALIBRATION_LIB_TARGET}
  openni_intrinsic_calibration
  openni_extrinsic_calibration
  openni_depth_ir_calibration
  openni_depth_calibration
  openni_calibration_check
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)