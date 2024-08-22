cmake_minimum_required(VERSION 3.0.2)
project(picarx_control)

find_package(catkin REQUIRED COMPONENTS  
  picarx_msgs
)

catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY
    tests/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()