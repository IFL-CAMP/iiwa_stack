cmake_minimum_required(VERSION 3.5)
project(iiwa_control)

find_package(catkin REQUIRED)
catkin_package()

foreach(dir config launch)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
