#!/usr/bin/env python3

import argparse

import rospy
from picarx.drivers.clutchgear import AbstractClutchGearDriver
from picarx_msgs.msg import ClutchGearStatus
from std_msgs.msg import Int8


class Options(object):
    """
    Parses command line arguments for the AckermannClutchGearDriver.

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


class AckermannClutchGearDriver(AbstractClutchGearDriver):
    """
    Driver for Ackermann steering mechanism with clutch gear.

    :param name: The RS232 device the simulator should send data to.
    :type name: str
    :param pwm_pin: The interval in which a line in the file should be read.
    :type pwm_pin: str
    :param i2c_port: The interval in which a line in the file should be read.
    :type i2c_port: str
    :param frequency: Frequency of the driver, defaults to 50.
    :type frequency: int, optional
    """

    def __init__(self, name: str, pwm_pin: str, i2c_port: str, frequency: int = 50, address: int = 20):
        super(AckermannClutchGearDriver, self).__init__(
            pwm_pin, i2c_port, address)
        self.name = name
        self.frequency = frequency
        self.command_subscriber = None
        self.status_publisher = None

    def rotate(self, ros_msgs: Int8):
        """
        Rotates the clutch gear based on the received ROS message.

        :param ros_msgs: ROS message containing the angle data.
        :type ros_msgs: Int8
        """
        angle = ros_msgs.data
        if angle <= 0:
            self.turn_left(angle)
        else:
            self.turn_right(angle)

    def turn_left(self, angle: int):
        """
        Turns the clutch gear to the left by the specified angle.

        :param angle: Angle to turn the clutch gear.
        :type angle: int
        """
        aimed_angle = angle + 90
        rate = rospy.Rate(50)
        for i in range(90, aimed_angle-1, -1):
            pulse_width = self.angle_to_pulse_width(i)
            self.pwm_pin.pulse_width = pulse_width
            rate.sleep()
        self.send_status()

    def turn_right(self, angle: int):
        """
        Turns the clutch gear to the right by the specified angle.

        :param angle: Angle to turn the clutch gear.
        :type angle: int
        """
        aimed_angle = angle + 90
        rate = rospy.Rate(50)
        for i in range(90, aimed_angle+1, 1):
            pulse_width = self.angle_to_pulse_width(i)
            self.pwm_pin.pulse_width = pulse_width
            rate.sleep()
        self.send_status()

    def send_status(self):
        """
        Sends the current status of the clutch gear.
        """
        current_pulse_width = self.pwm_pin.register_channel.read()
        self.status_publisher.publish(
            ClutchGearStatus(pulsewidth=current_pulse_width))

    def stop(self):
        """
        Stops the driver and logs the shutdown message.
        """
        rospy.loginfo("Shutting Ackermann steering driver down")

    def start(self):
        """
        Starts the driver and initializes ROS node and publishers.
        """
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
            options.args.name, options.args.pwm_pin, options.args.i2c_port, options.args.i2c_address)
        ackermann_clutchgear_driver.start()
    except rospy.ROSInterruptException:
        pass
