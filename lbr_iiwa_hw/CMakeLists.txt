cmake_minimum_required(VERSION 2.8.3)
project(lbr_iiwa_hw)


## TODO: need cleanup!!

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  roscpp
  message_generation
  tf
  urdf
  cmake_modules
)

add_definitions (-fpermissive)

################################################
## Declare ROS messages, services and actions ##
################################################

# Declare the message files to be built
add_message_files(FILES IIWAMsg.msg)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   IIWAMsg.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#  catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lwr_hw
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS 	message_runtime
			controller_interface
			hardware_interface
			control_toolbox
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(friremote
    src/friremote.cpp
    src/friudp.cpp
)

add_library(IIWARobot
    src/IIWARobot.cpp
    include/IIWARobot.h
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
    src/lwr_hw.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(lwr_hw_node lwr_hw_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    friremote
    IIWARobot
    ${catkin_LIBRARIES}
)

FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_INCLUDES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lwr_hw lwr_hw_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lwr_hw.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
