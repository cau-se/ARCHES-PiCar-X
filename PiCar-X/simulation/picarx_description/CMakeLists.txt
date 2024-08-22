cmake_minimum_required(VERSION 3.0.2)
project(picarx_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY materials meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
