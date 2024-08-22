#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from std_msgs.msg import Int8
from picarx.pwm import PWM
from picarx.gpio import Direction, GPIO


def change_direction_listener(self, event):
    pass

class TestDCMotors(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node("DCMOTORTEST")
        self.i2c_port = rospy.get_param('/motor_left/i2c_port', '')
        rospy.logerr("I2C PORT: " + self.i2c_port)


    def test_motor_left_forward(self):
        publisher = rospy.Publisher(
            '/motor_left/speed', Int8, queue_size=1)
        if publisher.get_num_connections() == 0:
            time.sleep(0.3)
        pwm_pin = PWM(channel='P12', i2c_port=self.i2c_port)    
        direction_pin = GPIO().setup(24, direction=Direction.OUT,
                                            callback=change_direction_listener, emulator=True)    
        publisher.publish(Int8(50)) # 110 degree for pwm
        time.sleep(0.5)
        pulse_width = pwm_pin.register_channel.read()
        self.assertEqual(direction_pin.value, 1)
        self.assertEqual(pulse_width, 2047)

    def test_motor_left_backward(self):
        publisher = rospy.Publisher(
            '/motor_left/speed', Int8, queue_size=1)
        if publisher.get_num_connections() == 0:
            time.sleep(0.3)
        pwm_pin = PWM(channel='P12', i2c_port=self.i2c_port)    
        direction_pin = GPIO().setup(24, direction=Direction.OUT,
                                            callback=change_direction_listener, emulator=True)    
        publisher.publish(Int8(-50)) # 110 degree for pwm
        time.sleep(0.5)
        pulse_width = pwm_pin.register_channel.read()
        self.assertEqual(direction_pin.value, 0)
        self.assertEqual(pulse_width, 2047)

    def test_motor_right_forward(self):
        publisher = rospy.Publisher(
            '/motor_right/speed', Int8, queue_size=1)
        if publisher.get_num_connections() == 0:
            time.sleep(0.3)
        pwm_pin = PWM(channel='P13', i2c_port=self.i2c_port)     
        direction_pin = GPIO().setup(23, direction=Direction.OUT,
                                            callback=change_direction_listener, emulator=True)      
        publisher.publish(Int8(50)) # 110 degree for pwm
        time.sleep(0.5)
        pulse_width = pwm_pin.register_channel.read()
        self.assertEqual(direction_pin.value, 0)
        self.assertEqual(pulse_width, 2047)

    def test_motor_right_backward(self):
        publisher = rospy.Publisher(
            '/motor_right/speed', Int8, queue_size=1)
        if publisher.get_num_connections() == 0:
            time.sleep(0.3)
        pwm_pin = PWM(channel='P13', i2c_port=self.i2c_port)     
        direction_pin = GPIO().setup(23, direction=Direction.OUT,
                                            callback=change_direction_listener, emulator=True)      
        publisher.publish(Int8(-50)) # 110 degree for pwm
        time.sleep(0.5)
        pulse_width = pwm_pin.register_channel.read()
        self.assertEqual(direction_pin.value, 1)
        self.assertEqual(pulse_width, 2047)

if __name__ == '__main__':
    rostest.rosrun(
        "picarx_dcmotor_driver", 'test_driving', TestDCMotors)
