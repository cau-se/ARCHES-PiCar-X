#!/usr/bin/env python3
#
# Copyright 2023 Alexander Barbie
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Any
from picarx_msgs.msg import DriveStatus, MotorStatus, ClutchGearStatus
import rospy
from picarx.interfaces.actuators import SunFounderClutchGear
from std_msgs.msg import Float64
import argparse
import math


class MonitorOptions(object):

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The name of the digital shadow skill")
        parser.add_argument(
            "uid", type=str, help="The unique identifier of the digital shadow skill. Leave blank if you want and automated generation.")

        self.args = parser.parse_args(argv)


class AckermannDriveDSNode(object):

    def __init__(self, name: str = 'MonitorSkill', uid: str = None) -> None:
        self.motor_left_publisher = None
        self.motor_right_publisher = None
        self.left_steer = None
        self.right_steer = None
        self.velocity = 15
        self.wheel_base = None
        self.wheel_track = None
        self.DEFAULT_PERIOD = 4095

    def start(self) -> None:
        rospy.init_node("Digital Shadow Status Monitoring", anonymous=False)
        rospy.on_shutdown(self.stop)

        self.wheel_base = float(rospy.get_param("~wheel_base"))
        self.wheel_track = float(rospy.get_param("~wheel_track"))
        # Subscribe to all status topics from the PT
        rospy.Subscriber(rospy.get_param('~status_topic'),
                         DriveStatus, self.send_to_model)

        # Publisher for the left motor in gazebo
        self.motor_left_publisher = rospy.Publisher(
            rospy.get_param('~motor_left_topic'), Float64, queue_size=5)
        # Publisher for the right motor in gazebo
        self.motor_right_publisher = rospy.Publisher(
            rospy.get_param('~motor_right_topic'), Float64, queue_size=5)

        # We were not able to rebuild the Ackermann steering with the steering bar thus each front wheel
        # has is own joint that has to be turned to steer.
        self.left_steer = rospy.Publisher(rospy.get_param(
            '~left_steer_topic'), Float64, queue_size=5)
        self.right_steer = rospy.Publisher(rospy.get_param(
            '~right_steer_topic'), Float64, queue_size=5)

        rospy.spin()

    def stop(self):
        rospy.loginfo("SHUTTING DOWN")

    def send_to_model(self, ros_msg: Any) -> None:
        rospy.loginfo(ros_msg)
        motor_left: MotorStatus = ros_msg.motor_left
        motor_right: MotorStatus = ros_msg.motor_right
        steering_clutchgear: ClutchGearStatus = ros_msg.clutchgear

        self.rotate(steering_clutchgear.pulsewidth)
        self.motor_left_publisher.publish(
            self.drive_with_speed(motor_left.direction, motor_left.speed))
        self.motor_right_publisher.publish(
            self.drive_with_speed(motor_right.direction, motor_right.speed))

    def pulse_width_to_angle(self, pulse_width):
        pulse_width = round(
            pulse_width * 2 * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value / self.DEFAULT_PERIOD)
        angle = (pulse_width - SunFounderClutchGear.MINIMUM_PULSE.value) * 180 / \
            (SunFounderClutchGear.MAXIMUM_PULSE.value -
             SunFounderClutchGear.MINIMUM_PULSE.value)
        return round(angle)

    def rotate(self, pulse_width):
        if pulse_width == 0:  # this is required since SunFounder adds a constant > 0 in the pulse width calculation for the angle. Thus the pulse width of an angle is always > 0.
            return
        angle = self.pulse_width_to_angle(pulse_width)
        rospy.loginfo(angle)
        angle = 90 - angle if angle >= 90 else 90 - angle

        if angle > 20:
            angle = 20
        elif angle < -20:
            angle = -20

        inside_wheel = self.angle_inside_wheel(angle)
        outside_wheel = self.angle_outside_wheel(angle)

        if angle > 0:
            self.turn_left(inside_wheel, outside_wheel)
        elif angle < 0:
            self.turn_right(inside_wheel, outside_wheel)
        else:
            self.left_steer.publish(0)
            self.right_steer.publish(0)

    def angle_inside_wheel(self, angle) -> float:
        alpha_inside = math.atan(
            self.wheel_base / (self.turning_radius(angle) - self.wheel_track/2))
        return alpha_inside

    def angle_outside_wheel(self, angle) -> float:
        alpha_outside = math.atan(
            self.wheel_base / (self.turning_radius(angle) + self.wheel_track/2))
        return alpha_outside

    def turn_right(self, inside_wheel, outside_wheel):
        self.left_steer.publish(outside_wheel)
        self.right_steer.publish(inside_wheel)

    def turn_left(self, inside_wheel, outside_wheel):
        self.left_steer.publish(inside_wheel)
        self.right_steer.publish(outside_wheel)

    def turning_radius(self, angle):
        if angle == 0:
            return 0
        turning_radius = self.wheel_base / math.tan(math.radians(angle))
        return turning_radius

    def drive_with_speed(self, direction, i2c_value: int):
        percentage = int((i2c_value/4095) * 100)
        return Float64(direction * self.velocity * percentage/100)


if __name__ == "__main__":
    start_arguments = MonitorOptions(rospy.myargv()[1:])
    driving_skill = AckermannDriveDSNode(
        start_arguments.args.name, start_arguments.args.uid)
    driving_skill.start()
