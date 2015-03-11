cmake_minimum_required(VERSION 2.8.3)
project(lbr_iiwa_ros)

find_package(catkin REQUIRED COMPONENTS
  IIWA
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
	include/IIWARos.h
	src/IIWARos.cpp
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}-example
    src/main.cpp
)

## Add dependence to the lbr_iiwa_msg module
add_dependencies(${PROJECT_NAME}-example 
				IIWA_generate_messages
				IIWA_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-example
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})