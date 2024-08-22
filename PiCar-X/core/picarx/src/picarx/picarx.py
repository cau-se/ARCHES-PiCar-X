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

from picarx.interfaces.actuators import TravelDirection, MotorSide
from picarx.drivers.clutchgear import Servo
from picarx.drivers.dc_motor import DCMotor
import time


class Picarx(object):
    """
    Class representing the PiCar-X robot.
    """
    PERIOD = 4095
    PRESCALER = 10

    def __init__(self):
        """
        Initialize the PiCar-X with its motors and steering servo.
        """
        self.left_motor = DCMotor(
            name='left_motor', direction_pin=24, pwm_pin="P12", motor_side=MotorSide.LEFT)
        self.right_motor = DCMotor(
            name='right_motor', direction_pin=23, pwm_pin="P13", motor_side=MotorSide.RIGHT)
        self.steering_servo = Servo(pwm_pin="P2")

    def steer(self, angle):
        """
        Steer the car to a specific angle.

        :param angle: The angle to steer.
        """
        self.steering_servo.rotate_by_angle(angle)

    def steer_left(self, angle):
        """
        Steer the car to the left by a specific angle.

        :param angle: The angle to steer to the left.
        """
        if angle < 0:
            angle = 0
        elif angle > 40:
            angle = 40

        for i in range(0, angle):
            self.steering_servo.rotate_by_angle(90 - i)
            time.sleep(0.01)

    def steer_right(self, angle):
        """
        Steer the car to the right by a specific angle.

        :param angle: The angle to steer to the right.
        """
        if angle < 0:
            angle = 0
        elif angle > 40:
            angle = 40

        for i in range(0, angle):
            self.steering_servo.rotate_by_angle(90 + i)
            time.sleep(0.01)

    def steer_duty_cycle(self, angle):
        """
        Steer the car using duty cycle.

        :param angle: The angle to steer.
        """
        self.steering_servo.rotate_by_duty_cacly(angle)

    def steer_neutral(self):
        """
        Set the steering to the neutral position.
        """
        self.steering_servo.rotate_by_angle(90)

    def forward(self, speed=50, time=None):
        """
        Move the car forward.

        :param speed: The speed to move forward.
        :param time: The duration to move forward.
        """
        self.left_motor.direction = TravelDirection.FORWARD
        self.right_motor.direction = TravelDirection.FORWARD
        self.left_motor.speed = speed
        self.right_motor.speed = speed

    def backward(self, speed=50, time=None):
        """
        Move the car backward.

        :param speed: The speed to move backward.
        :param time: The duration to move backward.
        """
        self.left_motor.direction = TravelDirection.BACKWARD
        self.right_motor.direction = TravelDirection.BACKWARD
        self.left_motor.speed = speed
        self.right_motor.speed = speed

    def stop(self):
        """
        Stop the car.
        """
        self.left_motor.stop()
        self.right_motor.stop()


if __name__ == "__main__":
    px = Picarx()
    px.forward(speed=20)
    time.sleep(5)
    print("hab stop gemacht")
    px.stop()
    time.sleep(2)
    px.backward(speed=50)
    time.sleep(5)
    px.stop()
    time.sleep(2)
    print('links')
    px.steer_left(40)
    print('stop')
    time.sleep(2)
    print('rechts')
    px.steer_right(120)
    print('stop')
    time.sleep(2)
    print('neutral')
    px.steer_neutral()
    print('stop')
    time.sleep(5)
    print('ende')
    # px.stop()