#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from std_msgs.msg import Int8
from picarx.pwm import PWM

class TestAckermannClutchGear(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node("ACKERMANNCLUTCHGEARTEST")
        self.publisher = rospy.Publisher(
            '/picarx/steer', Int8, queue_size=1)
        if self.publisher.get_num_connections() == 0:
            time.sleep(0.3)
        self.pwm_pin = PWM(channel='P2', i2c_port='/dev/i2c-0')

    def test_turn_right(self):
        self.publisher.publish(Int8(20)) # 110 degree for pwm
        time.sleep(0.5)
        pulse_width = self.pwm_pin.register_channel.read()
        self.assertEqual(pulse_width, 176)

    def test_turn_left(self):
        self.publisher.publish(Int8(-10)) # 80 degree for pwm
        time.sleep(0.5)
        pulse_width = self.pwm_pin.register_channel.read()
        self.assertEqual(pulse_width, 142)

    def test_straight(self):
        self.publisher.publish(Int8(0)) # 90 degree for pwm
        time.sleep(0.5)
        pulse_width = self.pwm_pin.register_channel.read()
        self.assertEqual(pulse_width, 154)


if __name__ == '__main__':
    rostest.rosrun(
        "picarx_clutchgear_driver", 'test_steering', TestAckermannClutchGear)
