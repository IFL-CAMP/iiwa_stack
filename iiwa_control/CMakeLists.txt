cmake_minimum_required(VERSION 3.5)
project(iiwa_control)

find_package(catkin REQUIRED)
catkin_package()

foreach(dir config launch)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()

catkin_install_python(PROGRAMS nodes/iiwa_sunrise.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})