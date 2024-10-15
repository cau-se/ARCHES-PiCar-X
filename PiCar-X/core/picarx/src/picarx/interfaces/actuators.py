#!/usr/bin/env python3
from abc import ABCMeta, abstractmethod
from enum import Enum
from typing import Union


class TravelDirection(Enum):
    FORWARD = 0
    BACKWARD = 1


class MotorSide(Enum):
    LEFT = 0
    RIGHT = 1


class SunFounderClutchGear(Enum):
    PULSE_PERIOD = 2000  # Every 20ms
    MIN_ANGLE = 0  # Degree
    MAX_ANGLE = 180  # Degree
    NEUTRAL_PULSE_LENGTH = 1500  # 1.5ms, will turn to 90 degree
    MINIMUM_PULSE = 500  # 0.5ms
    MAXIMUM_PULSE = 2500  # 2.5ms
    SUNFOUNDER_RANDOM_DIVIDER = 20000


class DCMotorInterface(metaclass=ABCMeta):
    """
    Interface for a DC motor.

    Methods:
    - __init__(name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], motor_side: MotorSide = MotorSide.LEFT)
    - name: str
    - direction_pin: Union[int, str]
    - pwm_pin: Union[int, str]
    - motor_side: MotorSide
    - direction: TravelDirection
    - speed: int
    - stop()
    """

    @abstractmethod
    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], motor_side: MotorSide = MotorSide.LEFT):
        """
        Initializes the DC motor.

        Args:
            name (str): Name of the motor.
            direction_pin (Union[int, str]): Pin for direction control.
            pwm_pin (Union[int, str]): Pin for PWM control.
            motor_side (MotorSide): Side of the motor (left or right).
        """
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def name(self):
        """Returns the name of the motor."""
        raise NotImplementedError("Subclasses should implement this!")

    @name.setter
    @abstractmethod
    def name(self, name):
        """Sets the name of the motor."""
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def direction_pin(self):
        """Returns the pin for direction control."""
        raise NotImplementedError("Subclasses should implement this!")

    @direction_pin.setter
    @abstractmethod
    def direction_pin(self, pin_number):
        """Sets the pin for direction control."""
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def pwm_pin(self):
        """Returns the pin for PWM control."""
        raise NotImplementedError("Subclasses should implement this!")

    @pwm_pin.setter
    @abstractmethod
    def pwm_pin(self, pin_number):
        """Sets the pin for PWM control."""
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def motor_side(self):
        """Returns the side of the motor (left or right)."""
        raise NotImplementedError("Subclasses should implement this!")

    @motor_side.setter
    @abstractmethod
    def motor_side(self, motor_side: MotorSide):
        """Sets the side of the motor (left or right)."""
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def direction(self):
        """Returns the travel direction of the motor."""
        raise NotImplementedError("Subclasses should implement this!")

    @direction.setter
    @abstractmethod
    def direction(self, direction: TravelDirection):
        """Sets the travel direction of the motor."""
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def speed(self):
        """Returns the speed of the motor."""
        raise NotImplementedError("Subclasses should implement this!")

    @speed.setter
    @abstractmethod
    def speed(self, speed: int):
        """Sets the speed of the motor."""
        raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def stop(self):
        """Stops the motor."""
        raise NotImplementedError("Subclasses should implement this!")


class ClutchGearInterface(metaclass=ABCMeta):
    """
    Interface for a clutch gear.

    Methods:
    - __init__(pwm_pin, i2c_port)
    - pwm_pin: Union[int, str]
    - angle: Union[int, float]
    - angle_to_pulse_width(angle)
    - pulse_width_to_angle(pulse_width)
    - rotate(angle)
    - start()
    - stop()
    """

    @abstractmethod
    def __init__(self, pwm_pin, i2c_port):
        """
        Initializes the clutch gear.

        Args:
            pwm_pin (Union[int, str]): Pin for PWM control.
            i2c_port: I2C port for communication.
        """
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def pwm_pin(self):
        """Returns the pin for PWM control."""
        raise NotImplementedError("Subclasses should implement this!")

    @pwm_pin.setter
    @abstractmethod
    def pwm_pin(self, pin_number):
        """Sets the pin for PWM control."""
        raise NotImplementedError("Subclasses should implement this!")

    @property
    @abstractmethod
    def angle(self):
        """Returns the angle of the clutch gear."""
        raise NotImplementedError("Subclasses should implement this!")

    @angle.setter
    @abstractmethod
    def angle(self, angle: Union[int, float]):
        """Sets the angle of the clutch gear."""
        raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def angle_to_pulse_width(self, angle):
        """
        Converts an angle to a pulse width.

        Args:
            angle (Union[int, float]): The angle in degrees.

        Returns:
            int: The corresponding pulse width.
        """
        raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def pulse_width_to_angle(self, pulse_width):
        """
        Converts a pulse width to an angle.

        Args:
            pulse_width (int): The pulse width.

        Returns:
            Union[int, float]: The corresponding angle in degrees.
        """
        raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def rotate(self, angle):
        """
        Rotates the clutch gear to the specified angle, 0 to 180 degree.

        Args:
            angle (Union[int, float]): The angle in degrees.
        """
        raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def start(self):
        """Starts the clutch gear."""
        raise NotImplementedError("Subclasses should implement this!")

    @abstractmethod
    def stop(self):
        """Stops the clutch gear."""
        raise NotImplementedError("Subclasses should implement this!")
