#!/usr/bin/env python3

import rospy
from typing import Union
from picarx.emulators.dcmotor import AbstractMotorEmulator
from picarx.interfaces.actuators import MotorSide, TravelDirection
import argparse
from std_msgs.msg import Float64


class Options(object):
    """
    Parses command line arguments for the DCMotorEmulator.

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
            "i2c_address", type=int, default=20, help="The register address on the I2C.")
        
        self.args = parser.parse_args(argv)

    def get_args(self):
        """
        Returns the parsed command line arguments.

        :return: Parsed command line arguments.
        :rtype: dict
        """
        return vars(self.args)


class DCMotorEmulator(AbstractMotorEmulator):
    """
    Emulator for DC motor mechanism.

    :param name: The RS232 device the simulator should send data to.
    :type name: str
    :param direction_pin: The path to a file, e.g., /path/to/file.txt.
    :type direction_pin: Union[int, str]
    :param pwm_pin: The interval in which a line in the file should be read.
    :type pwm_pin: Union[int, str]
    :param i2c_port: The interval in which a line in the file should be read.
    :type i2c_port: str
    :param motor_side: The side of the motor, defaults to MotorSide.LEFT.
    :type motor_side: MotorSide, optional
    """

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], i2c_port: str, motor_side: MotorSide = MotorSide.LEFT, address: int = 20):
        super(DCMotorEmulator, self).__init__(name, int(direction_pin),
                                              pwm_pin, i2c_port, MotorSide(int(motor_side)), address)
        self.direction_pin.callback = self.change_direction_listener
        self.frequency = 50
        self.pulse_width = 0
        self.controller_publisher = None
        self.velocity = 21

    @property
    def controller_publisher(self):
        """
        Gets the ROS publisher for the controller.

        :return: ROS publisher for the controller.
        :rtype: rospy.Publisher
        """
        return self.__joint_publisher

    @controller_publisher.setter
    def controller_publisher(self, publisher: rospy.Publisher):
        """
        Sets the ROS publisher for the controller.

        :param publisher: ROS publisher for the controller.
        :type publisher: rospy.Publisher
        :raises TypeError: If the publisher is not a rospy.Publisher instance.
        """
        if publisher is None:
            self.__joint_publisher = None
        else:
            if isinstance(publisher, rospy.Publisher):
                self.__joint_publisher = publisher
            else:
                raise TypeError(
                    "The Method joint_publisher expects a ROS Publisher object not the type: {}".format(type(publisher)))

    def change_direction_listener(self, event):
        """
        Listener for changes in the direction pin.

        :param event: Event indicating a change in the direction pin.
        :type event: Event
        """
        if event.event_type == 'modified':
            self.direction = TravelDirection(self.direction_pin.value)

    def drive_with_speed(self, speed):
        """
        Drives the motor with the specified speed.

        :param speed: Speed to drive the motor.
        :type speed: float
        """
        self.controller_publisher.publish(
            Float64(self.direction * self.velocity * speed / 100))

    def start(self):
        """
        Starts the emulator and initializes ROS node and publishers.
        """
        rospy.init_node(self.name, anonymous=True)
        rospy.on_shutdown(self.stop)
        frequency = rospy.Rate(50)

        self.controller_publisher = rospy.Publisher(rospy.get_param(
            "~controller_publisher_topic"), Float64, queue_size=5)

        while not rospy.is_shutdown():
            pin_value = self.direction_pin.value
            if pin_value == 0:
                self.direction = TravelDirection.FORWARD
            else:
                self.direction = TravelDirection.BACKWARD
                
            try:
                if self.direction is not None:
                    self.drive_with_speed(self.pwm_pin.duty_cycle)
            except Exception as e:
                rospy.logerr(e)
            frequency.sleep()

    def stop(self):
        """
        Stops the emulator and shuts down ROS.
        """
        rospy.loginfo("Shutting the Node down")


if __name__ == '__main__':
    options = Options(rospy.myargv()[1:])
    dcmotor_emulator = DCMotorEmulator(options.args.name, options.args.direction_pin,
                                       options.args.pwm_pin, options.args.i2c_port, options.args.motor_side, options.args.i2c_address)
    dcmotor_emulator.start()