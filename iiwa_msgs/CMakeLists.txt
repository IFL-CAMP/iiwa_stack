cmake_minimum_required(VERSION 3.5)
project(iiwa_msgs)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    geometry_msgs
    message_generation
    std_msgs)

# Declare the message files to be built
add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
add_action_files(DIRECTORY action)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime std_msgs ) 
