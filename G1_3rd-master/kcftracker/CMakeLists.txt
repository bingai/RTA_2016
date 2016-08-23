cmake_minimum_required(VERSION 2.8)
project(test)

find_package(OpenCV REQUIRED)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)

include_directories(include) 
FILE(GLOB_RECURSE sourcefiles "src/*.cpp")

set(LIB_NAME "kcftracker")
add_library (${LIB_NAME}  SHARED ${sourcefiles} )

install(TARGETS ${LIB_NAME} 
	LIBRARY DESTINATION /usr/local/lib
)

install(DIRECTORY include/
	DESTINATION /usr/local/include
)

#uninstall target
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
	IMMEDIATE @ONLY)


add_custom_target(uninstall
	COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

