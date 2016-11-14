cmake_minimum_required(VERSION 2.8.3)
project(iiwa_hw)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  combined_robot_hw
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  tf
  urdf
  cmake_modules
  iiwa_msgs
  iiwa_ros
  roscpp
  pluginlib
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES	 ${PROJECT_NAME}
 CATKIN_DEPENDS  controller_interface
   hardware_interface
   control_toolbox
   combined_robot_hw
   pluginlib
   roscpp
)

## Specify additional locations of header files
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  include/iiwa_hw.h
  src/iiwa_hw.cpp
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}-bin
    src/main.cpp
)

add_executable(iiwa_combined_hw_node src/iiwa_combined_hw_node.cpp)

## Add dependence to the iiwa_msg module for the executable
add_dependencies(${PROJECT_NAME}-bin
    iiwa_msgs_generate_messages_cpp)

add_dependencies(iiwa_combined_hw_node
    iiwa_msgs_generate_messages_cpp)

## Add dependence to the iiwa_msg module for the library
add_dependencies(${PROJECT_NAME}
    iiwa_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-bin
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

target_link_libraries(iiwa_combined_hw_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    add_rostest_gtest(combined_robot_hw_test
#        test/combined_robot_hw_test.test
#        test/combined_robot_hw_test.cpp
#    )
#    target_link_libraries(combined_robot_hw_test ${PROJECT_NAME} ${catkin_LIBRARIES})    
#endif()

install(TARGETS ${PROJECT_NAME}
                ${PROJECT_NAME}-bin
                iiwa_combined_hw_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_hw_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/ config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
