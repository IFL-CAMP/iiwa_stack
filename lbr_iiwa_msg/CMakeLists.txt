cmake_minimum_required(VERSION 2.8.3)
project(IIWA)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  cmake_modules
)

# Declare the message files to be built
add_message_files(FILES IIWAMsg.msg)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS 	message_runtime
)