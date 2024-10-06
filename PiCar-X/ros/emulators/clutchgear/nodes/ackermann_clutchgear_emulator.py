#!/usr/bin/env python3

import rospy
from picarx.emulators.clutchgear import AbstractClutchGearEmulator
from std_msgs.msg import Float64
import math
import argparse


class Options(object):
    """
    Parses command line arguments for the AckermannClutchGearEmulator.

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


class AckermannClutchGearEmulator(AbstractClutchGearEmulator):
    """
    Emulator for Ackermann steering mechanism with clutch gear.

    :param name: The RS232 device the simulator should send data to.
    :type name: str
    :param pwm_pin: The interval in which a line in the file should be read.
    :type pwm_pin: str
    :param i2c_port: The interval in which a line in the file should be read.
    :type i2c_port: str
    :param frequency: Frequency of the emulator, defaults to 50.
    :type frequency: int, optional
    """

    def __init__(self, name: str, pwm_pin: str, i2c_port: str, address: int = 20, frequency: int = 50):
        super(AckermannClutchGearEmulator, self).__init__(
            pwm_pin, i2c_port, address=address)
        self.name = name
        self.frequency = frequency
        self.left_steer = None
        self.right_steer = None
        self.wheel_base = None
        self.wheel_track = None

    def angle_inside_wheel(self, angle) -> float:
        """
        Calculates the angle of the inside wheel.

        :param angle: Steering angle.
        :type angle: float
        :return: Angle of the inside wheel.
        :rtype: float
        """
        alpha_inside = math.atan(
            self.wheel_base / (self.turning_radius(angle) - self.wheel_track/2))
        return alpha_inside

    def angle_outside_wheel(self, angle) -> float:
        """
        Calculates the angle of the outside wheel.

        :param angle: Steering angle.
        :type angle: float
        :return: Angle of the outside wheel.
        :rtype: float
        """
        alpha_outside = math.atan(
            self.wheel_base / (self.turning_radius(angle) + self.wheel_track/2))
        return alpha_outside

    def turning_radius(self, angle):
        """
        Calculates the turning radius based on the steering angle.

        :param angle: Steering angle.
        :type angle: float
        :return: Turning radius.
        :rtype: float
        """
        if angle == 0:
            return 0
        turning_radius = self.wheel_base / math.tan(math.radians(angle))
        return turning_radius

    def rotate(self, pulse_width):
        """
        Rotates the wheels based on the pulse width.

        :param pulse_width: Pulse width for the steering angle.
        :type pulse_width: float
        """
        if pulse_width == 0:
            return
        angle = self.pulse_width_to_angle(pulse_width)
        angle = 90 - angle if angle >= 90 else 90 - angle
        angle = angle

        inside_wheel = self.angle_inside_wheel(angle)
        outside_wheel = self.angle_outside_wheel(angle)

        if angle > 0:
            self.turn_left(inside_wheel, outside_wheel)
        elif angle < 0:
            self.turn_right(inside_wheel, outside_wheel)
        else:
            self.left_steer.publish(0)
            self.right_steer.publish(0)

    def turn_right(self, inside_wheel, outside_wheel):
        """
        Turns the wheels to the right.

        :param inside_wheel: Angle of the inside wheel.
        :type inside_wheel: float
        :param outside_wheel: Angle of the outside wheel.
        :type outside_wheel: float
        """
        self.left_steer.publish(outside_wheel)
        self.right_steer.publish(inside_wheel)

    def turn_left(self, inside_wheel, outside_wheel):
        """
        Turns the wheels to the left.

        :param inside_wheel: Angle of the inside wheel.
        :type inside_wheel: float
        :param outside_wheel: Angle of the outside wheel.
        :type outside_wheel: float
        """
        self.left_steer.publish(inside_wheel)
        self.right_steer.publish(outside_wheel)

    def stop(self):
        """
        Stops the emulator and logs the shutdown message.
        """
        rospy.loginfo("Shutting Ackermann steering emulator down")

    def start(self):
        """
        Starts the emulator and initializes ROS node and publishers.
        """
        rospy.init_node(self.name, anonymous=True)
        rospy.loginfo("Ackermann steering emulator initialized")
        rospy.on_shutdown(self.stop)
        self.left_steer = rospy.Publisher(rospy.get_param(
            '~left_steer_topic'), Float64, queue_size=5)
        self.right_steer = rospy.Publisher(rospy.get_param(
            '~right_steer_topic'), Float64, queue_size=5)
        self.wheel_base = float(rospy.get_param("~wheel_base"))
        self.wheel_track = float(rospy.get_param("~wheel_track"))
        frequency = rospy.Rate(50)  # default 50Hz
        while not rospy.is_shutdown():
            self.rotate(self.pwm_pin.pulse_width)
            frequency.sleep()


if __name__ == '__main__':
    try:
        options = Options(rospy.myargv()[1:])
        clutchgear_emulator = AckermannClutchGearEmulator(
            options.args.name, options.args.pwm_pin, options.args.i2c_port, options.args.i2c_address)
        clutchgear_emulator.start()
    except rospy.ROSInterruptException:
        pass
