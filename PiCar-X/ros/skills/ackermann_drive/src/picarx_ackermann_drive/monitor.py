#!/usr/bin/env python3

from typing import Union
import rospy
from arches_core.skills import Skill
from picarx_msgs.msg import DriveStatus
import argparse


class AckermannStartOptions(object):
    """Start argument parser to the digital shadow skill.
        You have to set the name and a unique id. If you leave out a unique id,
        an autoincrement value will be used as id.

        Note:
            If you use Docker, we recommend to set the ids.
    """

    def __init__(self, argv):
        """
        Initialize the argument parser with command line arguments.

        Args:
            argv (list): List of command line arguments.
        """
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The name of the digital shadow skill")
        parser.add_argument(
            "uid", type=str, help="The unique identifier of the digital shadow skill. Leave blank if you want and automated generation.")

        self.args = parser.parse_args(argv)


class AckermannMonitorSkill(Skill):
    """
    AckermannDriveSkill class for controlling the Ackermann drive.

    Attributes:
        motor_left_publisher (rospy.Publisher): Publisher for the left motor.
        motor_right_publisher (rospy.Publisher): Publisher for the right motor.
        clutchgear_publisher (rospy.Publisher): Publisher for the clutch gear.
    """

    def __init__(self, name: str, uid: str = None) -> None:
        """
        Initialize the AckermannDriveSkill.

        Args:
            name (str): Name of the skill.
            uid (str, optional): Unique identifier of the skill. Defaults to None.
        """
        super(AckermannMonitorSkill, self).__init__(name, 'Monitoring', uid)
        self.motor_left_publisher = None
        self.motor_right_publisher = None
        self.clutchgear_publisher = None

    @property
    def motor_left_publisher(self) -> Union[None, rospy.Publisher]:
        """
        Get the left motor publisher.

        Returns:
            Union[None, rospy.Publisher]: The left motor publisher.
        """
        return self.__motor_left_publisher

    @motor_left_publisher.setter
    def motor_left_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        """
        Set the left motor publisher.

        Args:
            publisher (Union[None, rospy.Publisher]): The left motor publisher.

        Raises:
            ValueError: If the publisher is not of type rospy.Publisher or None.
        """
        if publisher is None:
            self.__motor_left_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__motor_left_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def motor_right_publisher(self) -> Union[None, rospy.Publisher]:
        """
        Get the right motor publisher.

        Returns:
            Union[None, rospy.Publisher]: The right motor publisher.
        """
        return self.__motor_right_publisher

    @motor_right_publisher.setter
    def motor_right_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        """
        Set the right motor publisher.

        Args:
            publisher (Union[None, rospy.Publisher]): The right motor publisher.

        Raises:
            ValueError: If the publisher is not of type rospy.Publisher or None.
        """
        if publisher is None:
            self.__motor_right_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__motor_right_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def clutchgear_publisher(self) -> Union[None, rospy.Publisher]:
        """
        Get the clutch gear publisher.

        Returns:
            Union[None, rospy.Publisher]: The clutch gear publisher.
        """
        return self.__clutchgear_publisher

    @clutchgear_publisher.setter
    def clutchgear_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        """
        Set the clutch gear publisher.

        Args:
            publisher (Union[None, rospy.Publisher]): The clutch gear publisher.

        Raises:
            ValueError: If the publisher is not of type rospy.Publisher or None.
        """
        if publisher is None:
            self.__clutchgear_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__clutchgear_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    def drive_from_status(self, ros_msg: DriveStatus) -> None:
        """
        Drive the vehicle based on the received status message from the physical twin.

        Args:
            ros_msg (DriveStatus): The ROS message containing the drive status.
        """
        self.motor_left_publisher.publish(ros_msg.motor_left)
        self.motor_right_publisher.publish(ros_msg.motor_left)
        self.clutchgear_publisher.publish(ros_msg.clutchgear)

    def stop(self):
        """
        Stop the skill and log the shutdown.
        """
        rospy.loginfo("Shutting the Node down")
