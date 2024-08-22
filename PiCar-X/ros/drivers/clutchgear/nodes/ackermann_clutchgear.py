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

import argparse

import rospy
from picarx.drivers.clutchgear import AbstractClutchGearDriver
from picarx_msgs.msg import ClutchGearStatus
from std_msgs.msg import Int8


class Options(object):

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The RS232 device the simulator should send data to, e.g., /dev/com0")
        parser.add_argument(
            "pwm_pin", type=str, help="The interval in which a line in the file should be read.")
        parser.add_argument(
            "i2c_port", type=str, help="The interval in which a line in the file should be read.")

        self.args = parser.parse_args(argv)

    def get_args(self):
        return vars(self.args)


class AckermannClutchGearDriver(AbstractClutchGearDriver):

    def __init__(self, name: str, pwm_pin: str, i2c_port: str, frequency: int = 50):
        super(AckermannClutchGearDriver, self).__init__(pwm_pin, i2c_port)
        self.name = name
        self.frequency = frequency
        self.command_subscriber = None
        self.status_publisher = None

    def rotate(self, ros_msgs):
        angle = ros_msgs.data
        if angle <= 0:
            self.turn_left(angle)
        else:
            self.turn_right(angle)

    def turn_left(self, angle):
        aimed_angle = angle + 90
        rate = rospy.Rate(50)
        for i in range(90, aimed_angle-1, -1):
            pulse_width = self.angle_to_pulse_width(i)
            self.pwm_pin.pulse_width = pulse_width
            rate.sleep()
        self.send_status()

    def turn_right(self, angle):
        aimed_angle = angle + 90
        rate = rospy.Rate(50)
        for i in range(90, aimed_angle+1, 1):
            pulse_width = self.angle_to_pulse_width(i)
            self.pwm_pin.pulse_width = pulse_width
            rate.sleep()
        self.send_status()

    def send_status(self):
        current_pulse_width = self.pwm_pin.register_channel.read()
        # current_angle = self.pulse_width_to_angle(current_pulse_width)
        self.status_publisher.publish(
            ClutchGearStatus(pulsewidth=current_pulse_width))

    def stop(self):
        rospy.loginfo("Shutting Ackermann steering driver down")

    def start(self):
        rospy.init_node(self.name, anonymous=True)
        rospy.loginfo("Ackermann steering driver initialized")
        rospy.on_shutdown(self.stop)
        self.command_subscriber = rospy.Subscriber(
            rospy.get_param('~steering_topic'), Int8, callback=self.rotate)
        self.status_publisher = rospy.Publisher(
            rospy.get_param('~steering_status_topic'), ClutchGearStatus, queue_size=5)
        rospy.spin()


if __name__ == '__main__':
    try:
        options = Options(rospy.myargv()[1:])
        ackermann_clutchgear_driver = AckermannClutchGearDriver(
            options.args.name, options.args.pwm_pin, options.args.i2c_port)
        ackermann_clutchgear_driver.start()
    except rospy.ROSInterruptException:
        pass
