cmake_minimum_required(VERSION 2.8.3)
project(iiwa_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  cmake_modules
  roscpp
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS  
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
	src/iiwa_ros.cpp
)

add_executable(iiwa_command_test src/main.cpp) 
target_link_libraries(iiwa_command_test ${PROJECT_NAME} ${catkin_LIBRARIES})
			
## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME} iiwa_msgs_generate_messages_cpp)