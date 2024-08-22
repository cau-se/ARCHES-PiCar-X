cmake_minimum_required(VERSION 3.0.2)
project(picarx_ackermann_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  picarx_msgs
  roslaunch
  rostest
  rosunit
  arches_msgs
  arches_core
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY
    tests/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)

install(PROGRAMS
  nodes/ackermann_drive_skill.py
  nodes/ackermann_monitor_ds.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()