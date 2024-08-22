cmake_minimum_required(VERSION 3.0.2)
project(picarx_dcmotor_emulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  picarx_msgs
)


#catkin_python_setup()
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  nodes/dcmotor_emulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
