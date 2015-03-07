cmake_minimum_required(VERSION 2.8.3)
project(lbr_iiwa_hw)

find_package(catkin REQUIRED COMPONENTS
  IIWA
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  tf
  urdf
  cmake_modules
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS  controller_interface
			hardware_interface
			control_toolbox
)

## Specify additional locations of header files
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    src/lbr_iiwa_hw.cpp
    include/lbr_iiwa_hw.h
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}-bin
    src/main.cpp
)

## Add dependence to the lbr_iiwa_msg module
add_dependencies(${PROJECT_NAME}-bin IIWA_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-bin
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_INCLUDES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})