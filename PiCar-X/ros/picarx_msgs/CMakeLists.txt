cmake_minimum_required(VERSION 3.0.2)
project(picarx_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  arches_msgs
)

add_message_files(
	DIRECTORY msg
    FILES
    ClutchGearStatus.msg
    Drive.msg
    MotorStatus.msg
    DriveStatus.msg
)


generate_messages(
   DEPENDENCIES
   std_msgs
   arches_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
