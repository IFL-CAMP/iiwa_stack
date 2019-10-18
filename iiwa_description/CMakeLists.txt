cmake_minimum_required(VERSION 3.5)
project(iiwa_description)

find_package(catkin REQUIRED)
catkin_package()

# Generate urdfs from xacro data that are required for the MoveIt! setup assistant
message(STATUS "Generating ${PROJECT_SOURCE_DIR}/urdf/generated/iiwa7.urdf ...")
execute_process(COMMAND rosrun xacro xacro ${PROJECT_SOURCE_DIR}/urdf/iiwa7.urdf.xacro --inorder -o ${PROJECT_SOURCE_DIR}/urdf/generated/iiwa7.urdf OUTPUT_QUIET)

message(STATUS "Generating ${PROJECT_SOURCE_DIR}/urdf/generated/iiwa14.urdf ...")
execute_process(COMMAND rosrun xacro xacro ${PROJECT_SOURCE_DIR}/urdf/iiwa14.urdf.xacro --inorder -o ${PROJECT_SOURCE_DIR}/urdf/generated/iiwa14.urdf OUTPUT_QUIET)
