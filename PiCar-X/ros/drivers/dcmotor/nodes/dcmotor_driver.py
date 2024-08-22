#!/usr/bin/env python3
# Copyright 2022 - 2024 Alexander Barbie
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

import rospy
from typing import Union
from picarx.drivers.dcmotor import AbstractDCMotorDriver
from picarx.interfaces.actuators import MotorSide, TravelDirection
from std_msgs.msg import Int8
from picarx.gpio import GPIO
from picarx_msgs.msg import MotorStatus
import argparse
import time


class Options(object):

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The RS232 device the simulator should send data to, e.g., /dev/com0")
        parser.add_argument(
            "direction_pin", type=str, help="The path to a file, e.g., /path/to/file.txt")
        parser.add_argument(
            "pwm_pin", type=str, help="The interval in which a line in the file should be read.")
        parser.add_argument(
            "i2c_port", type=str, help="The interval in which a line in the file should be read.")
        parser.add_argument(
            "motor_side", type=str, help="The interval in which a line in the file should be read.")

        self.args = parser.parse_args(argv)

    def get_args(self):
        return vars(self.args)


class DCMotorDriver(AbstractDCMotorDriver):

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], i2c_port: str, motor_side: MotorSide) -> None:
        super(DCMotorDriver, self).__init__(name, int(direction_pin),
                                            pwm_pin, i2c_port, MotorSide(int(motor_side)))
        self.status_publisher = None

    def drive(self, ros_msg):
        speed = int(ros_msg.data)
        self.direction = TravelDirection.FORWARD if speed >= 0 else TravelDirection.BACKWARD
        self.speed = abs(speed)
        time.sleep(0.05)
        self.send_status()

    def send_status(self):
        status_message = MotorStatus(
            location=self.motor_side.value, direction=self.direction.value, speed=self.speed)
        self.status_publisher.publish(status_message)

    def start(self):
        rospy.init_node(self.name, anonymous=False)
        rospy.on_shutdown(self.stop)
        rospy.Subscriber(rospy.get_param('~command_topic'), Int8, self.drive)
        self.status_publisher = rospy.Publisher(
            rospy.get_param('~status_topic'), MotorStatus, queue_size=5)
        rospy.spin()

    def stop(self):
        GPIO().stop()
        rospy.loginfo("SHUTTING DOWN")


if __name__ == "__main__":
    options = Options(rospy.myargv()[1:])
    dcmotor = DCMotorDriver(options.args.name, options.args.direction_pin,
                            options.args.pwm_pin, options.args.i2c_port, options.args.motor_side)
    dcmotor.start()
