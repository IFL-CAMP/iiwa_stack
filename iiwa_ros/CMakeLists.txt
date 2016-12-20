cmake_minimum_required(VERSION 3.2.0)
project(iiwa_ros)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  cmake_modules
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
	src/iiwaRos.cpp
	src/iiwa_services.cpp
)
			
## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME} iiwa_msgs_generate_messages_cpp)