from abc import abstractmethod
from typing import Union

from picarx.interfaces.actuators import DCMotorInterface, TravelDirection, MotorSide
from picarx.gpio import Direction, GPIO
from picarx.pwm import PWM


class AbstractDCMotorDriver(DCMotorInterface):
    """
    Abstract base class for DC motor drivers.
    """

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], i2c_port: str = '/dev/i2c-1', motor_side: MotorSide = MotorSide.LEFT):
        """
        Initialize the DC motor driver.

        :param name: The name of the motor.
        :param direction_pin: The GPIO pin for direction control.
        :param pwm_pin: The PWM pin configuration.
        :param i2c_port: The I2C port configuration.
        :param motor_side: The side of the motor (left or right).
        """
        self.name = name
        self.direction_pin = direction_pin
        self.pwm_pin = {'channel': pwm_pin, 'i2c_port': i2c_port}
        self.motor_side = motor_side
        self.speed = 0
        self.direction = TravelDirection.FORWARD

    @property
    def name(self):
        """
        Get the name of the motor.

        :return: The name of the motor.
        """
        return self.__name

    @name.setter
    def name(self, name):
        """
        Set the name of the motor.

        :param name: The name of the motor.
        """
        self.__name = name

    @property
    def direction_pin(self):
        """
        Get the GPIO pin for direction control.

        :return: The GPIO pin for direction control.
        """
        return self.__direction_pin

    @direction_pin.setter
    def direction_pin(self, pin_number):
        """
        Set the GPIO pin for direction control.

        :param pin_number: The GPIO pin number.
        """
        self.__direction_pin = GPIO().setup(
            pin_number, direction=Direction.OUT, callback=None)

    @property
    def pwm_pin(self):
        """
        Get the PWM pin configuration.

        :return: The PWM pin configuration.
        """
        return self.__pwm_pin

    @pwm_pin.setter
    def pwm_pin(self, config: dict):
        """
        Set the PWM pin configuration.

        :param config: The PWM pin configuration.
        """
        self.__pwm_pin = PWM(
            channel=config['channel'], i2c_port=config['i2c_port'])
        self.__pwm_pin.period = 4095
        self.__pwm_pin.prescaler = 8

    @property
    def motor_side(self):
        """
        Get the side of the motor (left or right).

        :return: The side of the motor.
        """
        return self.__motor_side

    @motor_side.setter
    def motor_side(self, motor_side: MotorSide):
        """
        Set the side of the motor (left or right).

        The motor side is relevant, since the PiCar-X has to equal motors, but they are mirrored and hence, mounted in different directions.

        :param motor_side: The side of the motor.
        """
        self.__motor_side = motor_side

    @property
    def direction(self):
        """
        Get the travel direction of the motor.

        :return: The travel direction of the motor.
        """
        return self.__direction

    @direction.setter
    def direction(self, direction: TravelDirection):
        """
        Set the travel direction of the motor.
        
        The direction is set by setting the value of the direction pin to high (1) or low (0). Forward is high, backward is low.
        Since the motors are mirrored, forward and backward are reversed for the left and right motor.

        :param direction: The travel direction of the motor.
        """
        self.__direction = direction
        if direction.value != self.motor_side.value:
            self.direction_pin.on()  # forward
        else:
            self.direction_pin.off()  # backward

    @property
    def speed(self):
        """
        Get the speed of the motor.

        :return: The speed of the motor.
        """
        return self.pwm_pin.pulse_width

    @speed.setter
    def speed(self, speed: int):
        """
        Set the speed of the motor. Speed is a percentage value between 0 and 100. 
        Right now, there is bug for speed values between 1 and 15. In that case the speed is set to 15 by default.

        :param speed: The speed of the motor.
        """

        if speed in range(1, 15):
            speed = 15
            print("Speed must be 0 or between 15 and 100, you entered {}".format(speed))

        if speed > 100:
            speed = 100
            print("Speed must be 0 or between 15 and 100, you entered {}".format(speed))

        self.pwm_pin.duty_cycle = speed

    def drive_with_speed(self, speed: int):
        """
        Drive the motor with a given speed.

        :param speed: The speed of the motor.
        """
        self.speed = speed

    @abstractmethod
    def start(self):
        """
        Start the motor. Use some kind of thread to keep the dc motor running, e.g. with twisted or asyncio.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('start'))

    @abstractmethod
    def stop(self):
        """
        Stop the motor.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('stop'))