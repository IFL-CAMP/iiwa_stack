cmake_minimum_required(VERSION 2.8.3)
project(iiwa_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  tf
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
	include/iiwaRos.h
	src/iiwaRos.cpp
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}-example
    src/main.cpp
)

## Add dependence to the iiwa_msg module for the executable
add_dependencies(${PROJECT_NAME}-example
				iiwa_msgs_generate_messages_cpp)
				
## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME}
				iiwa_msgs_generate_messages_cpp)
				
				
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-example
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})