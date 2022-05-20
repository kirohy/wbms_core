cmake_minimum_required(VERSION 3.0.2)
project(hrpsys_wbms)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

