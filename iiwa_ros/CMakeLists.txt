cmake_minimum_required(VERSION 3.5)
project(iiwa_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  cmake_modules
  roscpp
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS iiwa_msgs roscpp
)

add_library(${PROJECT_NAME}
	src/iiwa_ros.cpp

  src/state/cartesian_pose.cpp
  src/state/cartesian_wrench.cpp
  src/state/joint_position.cpp
  src/state/joint_torque.cpp
  src/state/external_joint_torque.cpp
  src/state/joint_velocity.cpp
  src/state/destination_reached.cpp

  src/command/generic_command.cpp
  src/command/cartesian_pose.cpp
  src/command/cartesian_pose_linear.cpp
  src/command/joint_position.cpp
  src/command/joint_position_velocity.cpp
  src/command/joint_velocity.cpp

  src/service/control_mode.cpp
  src/service/path_parameters.cpp
  src/service/path_parameters_lin.cpp
  src/service/time_to_destination.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(iiwa_ros_test src/main.cpp)
target_include_directories(iiwa_ros_test PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(iiwa_ros_test ${PROJECT_NAME})

## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME} iiwa_msgs_generate_messages_cpp)
