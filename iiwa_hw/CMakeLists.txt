cmake_minimum_required(VERSION 3.5)
project(iiwa_hw)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  urdf
  cmake_modules
  iiwa_msgs
  iiwa_ros
  pluginlib
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
  CATKIN_DEPENDS
      iiwa_ros
      iiwa_msgs
      pluginlib
      controller_interface
      controller_manager
			hardware_interface
			control_toolbox
)

add_library(${PROJECT_NAME} src/iiwa_hw.cpp)
add_executable(${PROJECT_NAME}-bin src/main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}-bin PRIVATE include ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}-bin ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} iiwa_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}-bin iiwa_msgs_generate_messages_cpp)

