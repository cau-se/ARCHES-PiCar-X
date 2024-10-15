#!/usr/bin/env python3

import rospy
from picarx_msgs.msg import DriveStatus, MotorStatus, ClutchGearStatus
from picarx_ackermann_drive.monitor import AckermannMonitorSkill, AckermannStartOptions


class AckermanMonitorNode(AckermannMonitorSkill):

    def __init__(self, name: str = 'AckermannMonitorSkill', uid: str = None) -> None:
        super(AckermanMonitorNode, self).__init__(
            name, uid)

    def start(self) -> None:
        rospy.init_node("Ackermann driving Skill", anonymous=False)
        rospy.on_shutdown(self.stop)
        self.motor_left_publisher = rospy.Publisher(
            rospy.get_param('~motor_left_topic')+'/from_status', MotorStatus, queue_size=5)
        self.motor_right_publisher = rospy.Publisher(
            rospy.get_param('~motor_right_topic')+'/from_status', MotorStatus, queue_size=5)
        self.clutchgear_publisher = rospy.Publisher(
            rospy.get_param('~steering_topic')+'/from_status', ClutchGearStatus, queue_size=5)
        rospy.Subscriber(rospy.get_param('~status_topic'),
                         DriveStatus, self.drive_from_status)
        rospy.spin()


if __name__ == "__main__":
    start_arguments = AckermannStartOptions(rospy.myargv()[1:])
    driving_skill = AckermanMonitorNode(
        start_arguments.args.name, start_arguments.args.uid)
    driving_skill.start()
