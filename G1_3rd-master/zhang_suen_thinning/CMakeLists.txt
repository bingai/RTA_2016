cmake_minimum_required(VERSION 2.8)
project(zhang_suen_thinning)

# compile with C++11 support
add_definitions("-std=c++0x -Dlinux -O3")

find_package(OpenCV REQUIRED)

include_directories(include)

add_library(zhang_suen_thinning SHARED src/zhang_suen_thinning.cpp)
target_link_libraries(zhang_suen_thinning ${OpenCV_LIBRARIES})

set(CMAKE_SKIP_RPATH TRUE)

install(TARGETS zhang_suen_thinning
	LIBRARY DESTINATION /usr/local/lib
)

install(DIRECTORY include/
	DESTINATION /usr/local/include
)

# uninstall target
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
	IMMEDIATE @ONLY)

add_custom_target(uninstall
	COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
