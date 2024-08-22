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
from picarx.emulators.dcmotor import AbstractMotorEmulator
from picarx.interfaces.actuators import MotorSide, TravelDirection
import argparse
from std_msgs.msg import Float64


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


class DCMotorEmulator(AbstractMotorEmulator):

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], i2c_port: str, motor_side: MotorSide = MotorSide.LEFT):
        super(DCMotorEmulator, self).__init__(name, int(direction_pin),
                                              pwm_pin, i2c_port, MotorSide(int(motor_side)))
        self.direction_pin.callback = self.change_direction_listener
        self.frequency = 50
        self.pulse_width = 0
        self.controller_publisher = None
        self.velocity = 21

    @property
    def controller_publisher(self):
        return self.__joint_publisher

    @controller_publisher.setter
    def controller_publisher(self, publisher: rospy.Publisher):
        if publisher is None:
            self.__joint_publisher = None
        else:
            if isinstance(publisher, rospy.Publisher):
                self.__joint_publisher = publisher
            else:
                raise TypeError(
                    "The Method joint_publisher expects a ROS Publisher object not the type: {}".format(type(publisher)))

    def change_direction_listener(self, event):
        if event.event_type == 'modified':
            self.direction = TravelDirection(self.direction_pin.value)

    def drive_with_speed(self, i2c_value: int):
        percentage = int((i2c_value/4095) * 100)
        self.controller_publisher.publish(
            Float64(self.direction * self.velocity * percentage/100))

    def start(self):
        rospy.init_node(self.name, anonymous=True)
        frequency = rospy.Rate(50)

        self.controller_publisher = rospy.Publisher(rospy.get_param(
            "~controller_publisher_topic"), Float64, queue_size=5)

        while not rospy.is_shutdown():
            try:
                if self.direction is not None:
                    i2c_value = self.pwm_pin.register_channel.read()
                    self.drive_with_speed(i2c_value)
            except Exception as e:
                rospy.logerr(e)
            frequency.sleep()

    def stop(self):
        pass


if __name__ == '__main__':
    options = Options(rospy.myargv()[1:])
    dcmotor_emulator = DCMotorEmulator(options.args.name, options.args.direction_pin,
                                       options.args.pwm_pin, options.args.i2c_port, options.args.motor_side)
    dcmotor_emulator.start()
