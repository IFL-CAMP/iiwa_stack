cmake_minimum_required(VERSION 3.5)
project(iiwa_gazebo)

find_package(catkin REQUIRED)
catkin_package()

foreach(dir launch worlds)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
