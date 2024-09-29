#!/usr/bin/env python3

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
    """
    Parses command line arguments for the DCMotorDriver.

    :param argv: List of command line arguments.
    :type argv: list
    """

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
        parser.add_argument(
            "i2c_address", type=int, default=20,  help="The register address on the I2C.")

        self.args = parser.parse_args(argv)

    def get_args(self):
        """
        Returns the parsed command line arguments.

        :return: Parsed command line arguments.
        :rtype: dict
        """
        return vars(self.args)


class DCMotorDriver(AbstractDCMotorDriver):
    """
    Driver for DC motor mechanism.

    :param name: The RS232 device the simulator should send data to.
    :type name: str
    :param direction_pin: The path to a file, e.g., /path/to/file.txt.
    :type direction_pin: Union[int, str]
    :param pwm_pin: The interval in which a line in the file should be read.
    :type pwm_pin: Union[int, str]
    :param i2c_port: The interval in which a line in the file should be read.
    :type i2c_port: str
    :param motor_side: The side of the motor.
    :type motor_side: MotorSide
    """

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], i2c_port: str, motor_side: MotorSide, address: int = 20):
        """
        Initializes the DCMotorDriver.

        :param name: The RS232 device the simulator should send data to.
        :type name: str
        :param direction_pin: The path to a file, e.g., /path/to/file.txt.
        :type direction_pin: Union[int, str]
        :param pwm_pin: The interval in which a line in the file should be read.
        :type pwm_pin: Union[int, str]
        :param i2c_port: The interval in which a line in the file should be read.
        :type i2c_port: str
        :param motor_side: The side of the motor.
        :type motor_side: MotorSide
        """
        super(DCMotorDriver, self).__init__(name, int(direction_pin),
                                            pwm_pin, i2c_port, MotorSide(int(motor_side)), address)
        self.status_publisher = None

    def drive(self, ros_msg):
        """
        Drives the motor based on the received ROS message.

        :param ros_msg: ROS message containing the speed data.
        :type ros_msg: Int8
        """
        speed = int(ros_msg.data)
        self.direction = TravelDirection.FORWARD if speed >= 0 else TravelDirection.BACKWARD
        self.speed = abs(speed)
        time.sleep(0.05)
        self.send_status()

    def send_status(self):
        """
        Sends the current status of the motor.
        """
        status_message = MotorStatus(
            location=self.motor_side.value, direction=self.direction.value, speed=self.speed)
        self.status_publisher.publish(status_message)

    def start(self):
        """
        Starts the driver and initializes ROS node and publishers.
        """
        rospy.init_node(self.name, anonymous=False)
        rospy.on_shutdown(self.stop)
        rospy.Subscriber(rospy.get_param('~command_topic'), Int8, self.drive)
        self.status_publisher = rospy.Publisher(
            rospy.get_param('~status_topic'), MotorStatus, queue_size=5)
        rospy.spin()

    def stop(self):
        """
        Stops the driver and logs the shutdown message.
        """
        GPIO().stop()
        rospy.loginfo("SHUTTING DOWN")


if __name__ == "__main__":
    options = Options(rospy.myargv()[1:])
    dcmotor = DCMotorDriver(options.args.name, options.args.direction_pin,
                            options.args.pwm_pin, options.args.i2c_port, options.args.motor_side, options.args.i2c_address)
    dcmotor.start()
