cmake_minimum_required(VERSION 3.0.2)
project(picarx_gazebo)

cmake_policy(SET CMP0054 NEW)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(gazebo REQUIRED)

catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


install(DIRECTORY launch worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)