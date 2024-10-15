#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
from picarx_msgs.msg import Drive, DriveStatus, MotorStatus, ClutchGearStatus
from picarx_ackermann_drive.skill import AckermannDriveSkill, AckermannStartOptions
import message_filters


class AckermannDriveNode(AckermannDriveSkill):

    def __init__(self, name: str = 'AckermannSkill', uid: str = None) -> None:
        super(AckermannDriveNode, self).__init__(
            name, uid)

    def start(self) -> None:
        rospy.init_node("Ackermann driving Skill", anonymous=False)
        rospy.on_shutdown(self.stop)
        self.command_subscriber = rospy.Subscriber(
            rospy.get_param('~command_topic'), Drive, self.drive)
        self.motor_left_publisher = rospy.Publisher(
            rospy.get_param('~motor_left_topic'), Int8, queue_size=5)
        self.motor_right_publisher = rospy.Publisher(
            rospy.get_param('~motor_right_topic'), Int8, queue_size=5)
        self.clutchgear_publisher = rospy.Publisher(
            rospy.get_param('~steering_topic'), Int8, queue_size=5)
        self.status_publisher = rospy.Publisher(
            rospy.get_param('~status_topic'), DriveStatus, queue_size=5)

        self.init_status_filter(message_filters.Subscriber(rospy.get_param('~motor_left_status_topic'), MotorStatus), message_filters.Subscriber(
            rospy.get_param('~motor_right_status_topic'), MotorStatus), message_filters.Subscriber(rospy.get_param('~steering_status_topic'), ClutchGearStatus))

        rospy.spin()


if __name__ == "__main__":
    start_arguments = AckermannStartOptions(rospy.myargv()[1:])
    driving_skill = AckermannDriveNode(
        start_arguments.args.name, start_arguments.args.uid)
    driving_skill.start()
