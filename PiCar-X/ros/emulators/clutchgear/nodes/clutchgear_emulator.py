#!/usr/bin/env python3

from typing import Union
import rospy
from picarx.emulators.clutchgear import AbstractClutchGearEmulator
from std_msgs.msg import Float64
import argparse
import math


class Options(object):
    """
    Parses command line arguments for the ClutchGearEmulator.

    :param argv: List of command line arguments.
    :type argv: list
    """

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The RS232 device the simulator should send data to, e.g., /dev/com0")
        parser.add_argument(
            "pwm_pin", type=str, help="The interval in which a line in the file should be read.")
        parser.add_argument(
            "i2c_port", type=str, help="The interval in which a line in the file should be read.")
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


class ClutchGearEmulator(AbstractClutchGearEmulator):
    """
    Emulator for clutch gear mechanism.

    :param name: The RS232 device the simulator should send data to.
    :type name: str
    :param pwm_pin: The interval in which a line in the file should be read.
    :type pwm_pin: str
    :param i2c_port: The interval in which a line in the file should be read.
    :type i2c_port: str
    :param frequency: Frequency of the emulator, defaults to 50.
    :type frequency: int, optional
    """

    def __init__(self, name: str, pwm_pin: str, i2c_port: str, frequency: int = 50, address: int = 20):
        super(ClutchGearEmulator, self).__init__(
            pwm_pin, i2c_port, frequency, address)
        self.name = name
        self.frequency = frequency
        self.controller_publisher = None

    def rotate(self, angle):
        """
        Rotates the clutch gear based on the given angle.

        :param angle: Angle for the clutch gear rotation.
        :type angle: float
        """
        self.controller_publisher.publish(math.radians(angle))

    def stop(self):
        """
        Stops the emulator and logs the shutdown message.
        """
        rospy.loginfo("Shutting the Node down")
        rospy.shuutdown()

    def start(self):
        """
        Starts the emulator and initializes ROS node and publishers.
        """
        rospy.init_node(self.name, anonymous=True)
        rospy.loginfo("ClutchGear initialized")
        rospy.on_shutdown(self.stop)
        self.controller_publisher = rospy.Publisher(
            rospy.get_param('~clutchgear_topic'), Float64, queue_size=5)

        frequency = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            self.rotate(self.pwm_pin.pulse_width)
            frequency.sleep()


if __name__ == '__main__':
    try:
        options = Options(rospy.myargv()[1:])
        clutchgear_emulator = ClutchGearEmulator(
            options.args.name, options.args.pwm_pin, options.args.i2c_port, options.args.i2c_address)
        clutchgear_emulator.start()
    except rospy.ROSInterruptException:
        pass
