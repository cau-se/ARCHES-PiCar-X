from typing import Union
from picarx.gpio import Direction, GPIO
from picarx.pwm import PWM
from abc import ABCMeta, abstractmethod
from picarx.interfaces.actuators import MotorSide, TravelDirection


class AbstractMotorEmulator(metaclass=ABCMeta):
    """
    Abstract base class for motor emulators.
    
    The dcmotor emulator can be used for emulation purposes. The main interfaces are the GPIO and I2C interfaces. On unix systems, you need to active these modules.
    Therefore, type in the terminal the following command: 
        sudo modprobe gpio-mockup gpio_mockup_ranges=1,41
        sudo modprobe i2c-dev
        sudo modprobe i2c-stub chip_addr=0x14
    """

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], i2c_port: str = '/dev/i2c-1', motor_side: MotorSide = MotorSide.LEFT, address: int = 20):
        """
        Initialize the motor emulator.

        :param name: The name of the motor.
        :param direction_pin: The GPIO pin for direction control.
        :param pwm_pin: The PWM pin configuration.
        :param i2c_port: The I2C port configuration.
        :param motor_side: The side of the motor (left or right).
        """
        self.name = name
        self.direction_pin = direction_pin
        self.pwm_pin = {'channel': pwm_pin, 'i2c_port': i2c_port, 'address': address}
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
        self.__direction_pin = GPIO().setup(pin_number, direction=Direction.OUT,
                                            callback=self.change_direction_listener, emulator=True)

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
            channel=config['channel'], i2c_port=config['i2c_port'], address=config['address'])
        self.pwm_pin.period = 4095
        self.pwm_pin.prescaler = 8

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

        :param direction: The travel direction of the motor.
        """
        if direction.value != self.motor_side.value:
            self.__direction = 1  # forward
        else:
            self.__direction = -1  # backward

    @property
    def speed(self):
        """
        Get the speed of the motor.

        :return: The speed of the motor.
        """
        return  self.pwm_pin.duty_cycle

    @speed.setter
    def speed(self, speed: int):
        """
        Set the speed of the motor.

        :param speed: The speed of the motor.
        :raises ValueError: If the speed is not between 15 and 100.
        """
        
        if speed > 100:
            speed = 100
            
        if 0 < speed < 15:
            speed = 15
            print("Speed must be between 15 and 100, you entered {}".format(speed))

        self.pwm_pin.duty_cycle = speed

    @abstractmethod
    def change_direction_listener(self, event):
        """
        Abstract method to handle direction change events.

        :param event: The event that triggered the direction change.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('change_direction_listener'))

    @abstractmethod
    def drive_with_speed(self, speed_value: int):
        """
        Abstract method to drive the motor with a given speed. Use this function to send speed values to a simulation.

        :param speed_value: The I2C value representing the speed.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('drive_with_speed'))

    @abstractmethod
    def start(self):
        """
        Abstract method to start the motor.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('start'))

    @abstractmethod
    def stop(self):
        """
        Abstract method to stop the motor.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('stop'))