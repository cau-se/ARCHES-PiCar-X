#!/usr/bin/env python3

from picarx_msgs.msg import Drive
import rospy
from picarx.interfaces.actuators import SunFounderClutchGear
from std_msgs.msg import Float64
import argparse
import math
from typing import Union
from picarx.interfaces.actuators import SunFounderClutchGear



class DriveExecutorOptions(object):
    """
    Parses command line arguments for the AckermannDriveDSNode.

    :param argv: List of command line arguments.
    :type argv: list
    """

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The name of the digital shadow skill")
        parser.add_argument(
            "uid", type=str, help="The unique identifier of the digital shadow skill. Leave blank if you want an automated generation.")

        self.args = parser.parse_args(argv)

    def get_args(self):
        """
        Returns the parsed command line arguments.

        :return: Parsed command line arguments.
        :rtype: dict
        """
        return vars(self.args)

class DriveExecutorDM(object):
    """
    Node for monitoring the Ackermann drive status.

    :param name: The name of the digital shadow skill.
    :type name: str
    :param uid: The unique identifier of the digital shadow skill.
    :type uid: str, optional
    """

    def __init__(self, name: str = 'DriveExecutorDM', uid: str = None) -> None:
        self.motor_left_publisher = None
        self.motor_right_publisher = None
        self.left_steer = None
        self.right_steer = None
        self.velocity = 21
        self.wheel_base = None
        self.wheel_track = None
        self.DEFAULT_PERIOD = 4095
        

    def start(self) -> None:
        """
        Starts the ROS node and initializes publishers and subscribers.
        """
        rospy.init_node("Digital Shadow Status Monitoring", anonymous=False)
        rospy.on_shutdown(self.stop)

        self.wheel_base = float(rospy.get_param("~wheel_base"))
        self.wheel_track = float(rospy.get_param("~wheel_track"))
        # Subscribe to all status topics from the PT

        # Publisher for the left motor in gazebo
        self.motor_left_publisher = rospy.Publisher(
            rospy.get_param('~motor_left_topic'), Float64, queue_size=5)
        # Publisher for the right motor in gazebo
        self.motor_right_publisher = rospy.Publisher(
            rospy.get_param('~motor_right_topic'), Float64, queue_size=5)

        # We were not able to rebuild the Ackermann steering with the steering bar thus each front wheel
        # has its own joint that has to be turned to steer.
        self.left_steer = rospy.Publisher(rospy.get_param(
            '~left_steer_topic'), Float64, queue_size=5)
        self.right_steer = rospy.Publisher(rospy.get_param(
            '~right_steer_topic'), Float64, queue_size=5)
        rospy.Subscriber(
            rospy.get_param('~command_topic'), Drive, self.send_to_model)
        
        rospy.spin()

    def stop(self):
        """
        Stops the ROS node and logs the shutdown message.
        """
        rospy.loginfo("Shutting the Node down")

    def send_to_model(self, ros_msg: Drive) -> None:
        """
        Processes the received ROS message and sends commands to the model.

        :param ros_msg: ROS message containing the drive status.
        :type ros_msg: Any
        """
        direction = 0
        if ros_msg.speed < 0:
            direction = 1
        
        angle = ros_msg.angle
        if angle > 20:
            angle = 20
        elif angle < -20:
            angle = -20
        

        self.rotate(angle)
        self.motor_left_publisher.publish(
            self.drive_with_speed(abs(ros_msg.speed), 0, direction))
        self.motor_right_publisher.publish(
            self.drive_with_speed(abs(ros_msg.speed), 1, direction))

    def angle_to_pulse_width(self, angle):
        """
        Convert an angle to a pulse width.

        :param angle: The angle.
        :return: The pulse width.
        """
        pulse_width = round(angle / 180 * (SunFounderClutchGear.MAXIMUM_PULSE.value -
                            SunFounderClutchGear.MINIMUM_PULSE.value) + SunFounderClutchGear.MINIMUM_PULSE.value)
        pulse_width = pulse_width / \
            SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value * self.pwm_pin.period
        return round(pulse_width / 2)

    def pulse_width_to_angle(self, pulse_width: int) -> int:
        """
        Converts pulse width to angle.

        :param pulse_width: Pulse width value.
        :type pulse_width: int
        :return: Corresponding angle.
        :rtype: int
        """
        pulse_width = round(
            pulse_width * 2 * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value / self.DEFAULT_PERIOD)
        angle = (pulse_width - SunFounderClutchGear.MINIMUM_PULSE.value) * 180 / \
            (SunFounderClutchGear.MAXIMUM_PULSE.value -
             SunFounderClutchGear.MINIMUM_PULSE.value)
        return round(angle)



    def rotate(self, angle: int) -> None:
        """
        Rotates the steering based on the pulse width.

        :param pulse_width: Pulse width value.
        :type pulse_width: int
        """

        inside_wheel = self.angle_inside_wheel(angle)
        outside_wheel = self.angle_outside_wheel(angle)

        if angle > 0:
            self.turn_left(inside_wheel, outside_wheel)
        elif angle < 0:
            self.turn_right(inside_wheel, outside_wheel)
        else:
            self.left_steer.publish(0)
            self.right_steer.publish(0)

    def angle_to_pulse_width(self, angle):
        """
        Convert an angle to a pulse width.

        :param angle: The angle.
        :return: The pulse width.
        """
        pulse_width = round(angle / 180 * (SunFounderClutchGear.MAXIMUM_PULSE.value -
                            SunFounderClutchGear.MINIMUM_PULSE.value) + SunFounderClutchGear.MINIMUM_PULSE.value)
        pulse_width = pulse_width / \
            SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value * self.pwm_pin.period
        return round(pulse_width / 2)

    def angle_inside_wheel(self, angle: int) -> float:
        """
        Calculates the angle for the inside wheel.

        :param angle: Steering angle.
        :type angle: int
        :return: Angle for the inside wheel.
        :rtype: float
        """
        alpha_inside = math.atan(
            self.wheel_base / (self.turning_radius(angle) - self.wheel_track/2))
        return alpha_inside

    def angle_outside_wheel(self, angle: int) -> float:
        """
        Calculates the angle for the outside wheel.

        :param angle: Steering angle.
        :type angle: int
        :return: Angle for the outside wheel.
        :rtype: float
        """
        alpha_outside = math.atan(
            self.wheel_base / (self.turning_radius(angle) + self.wheel_track/2))
        return alpha_outside

    def turn_right(self, inside_wheel: float, outside_wheel: float) -> None:
        """
        Turns the wheels to the right.

        :param inside_wheel: Angle for the inside wheel.
        :type inside_wheel: float
        :param outside_wheel: Angle for the outside wheel.
        :type outside_wheel: float
        """
        self.left_steer.publish(outside_wheel)
        self.right_steer.publish(inside_wheel)

    def turn_left(self, inside_wheel: float, outside_wheel: float) -> None:
        """
        Turns the wheels to the left.

        :param inside_wheel: Angle for the inside wheel.
        :type inside_wheel: float
        :param outside_wheel: Angle for the outside wheel.
        :type outside_wheel: float
        """
        self.left_steer.publish(inside_wheel)
        self.right_steer.publish(outside_wheel)

    def turning_radius(self, angle: int) -> float:
        """
        Calculates the turning radius based on the steering angle.

        :param angle: Steering angle.
        :type angle: int
        :return: Turning radius.
        :rtype: float
        """
        if angle == 0:
            return 0
        turning_radius = self.wheel_base / math.tan(math.radians(angle))
        return turning_radius
        

    def drive_with_speed(self, percentage: int, location: int = 0, direction: int = 0) -> Float64:
        """
        Drives the motor with the specified speed.

        :param direction: Travel direction.
        :type direction: int
        :param i2c_value: I2C value representing the speed.
        :type i2c_value: int
        :return: Speed value as Float64.
        :rtype: Float64
        """
        if direction != location:
            direction = -1  # forward
        else:
            direction = 1  # backward
            
        return Float64(direction * self.velocity * percentage/100)


if __name__ == "__main__":
    start_arguments = DriveExecutorOptions(rospy.myargv()[1:])
    driving_skill = DriveExecutorDM(
        start_arguments.args.name, start_arguments.args.uid)
    driving_skill.start()