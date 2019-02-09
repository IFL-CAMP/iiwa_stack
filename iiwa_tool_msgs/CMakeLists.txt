cmake_minimum_required(VERSION 3.5)
project(iiwa_tool_msgs)

find_package(
	catkin REQUIRED COMPONENTS
		actionlib_msgs
		message_generation
		cmake_modules
)

# Declare the message files to be built
add_message_files(DIRECTORY msg)
add_action_files(DIRECTORY action)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
