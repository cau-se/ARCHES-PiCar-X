from abc import abstractmethod
from typing import Union
from picarx.interfaces.actuators import ClutchGearInterface, SunFounderClutchGear
from picarx.pwm import PWM
from twisted.internet import reactor
import time


class AbstractClutchGearEmulator(ClutchGearInterface):
    """
    Abstract base class for clutch gear emulators.
    
    The clutch gear emulators can be used for emulation purposes. The main interface is the I2C interface. On unix systems, the I2C interface is usually located at /dev/i2c-0 and has to be created using the i2c-stub module.
    Therefore, type in the terminal the following commands:
        sudo modprobe i2c-dev
        sudo modprobe i2c-stub chip_addr=0x14
    """

    def __init__(self, pwm_pin, i2c_port, frequency=50):
        """
        Initialize the clutch gear emulator.

        :param pwm_pin: The PWM pin configuration.
        :param i2c_port: The I2C port configuration.
        :param frequency: The frequency in Hertz.
        """
        self.pwm_pin = {'channel': pwm_pin, 'i2c_port': i2c_port}
        self.frequency = frequency  # in Hertz

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
    def frequency(self):
        """
        Get the frequency.

        :return: The frequency.
        """
        return self.__frequency

    @frequency.setter
    def frequency(self, frequency):
        """
        Set the frequency.

        :param frequency: The frequency in Hertz.
        """
        self.__frequency = 1/frequency

    @property
    def angle(self):
        """
        Get the angle.

        :return: The angle.
        """
        return self.__angle

    @angle.setter
    def angle(self, angle: Union[int, float]):
        """
        Set the angle.

        :param angle: The angle.
        """
        self.__angle = angle

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

    def pulse_width_to_angle(self, pulse_width):
        """
        Convert a pulse width to an angle.

        :param pulse_width: The pulse width.
        :return: The angle.
        """
        pulse_width = round(
            pulse_width * 2 * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value / self.pwm_pin.period)
        angle = (pulse_width - SunFounderClutchGear.MINIMUM_PULSE.value) * 180 / \
            (SunFounderClutchGear.MAXIMUM_PULSE.value -
             SunFounderClutchGear.MINIMUM_PULSE.value)
        return round(angle)

    def rotate_by_pulse_width(self, pulse_width):
        """
        Rotate by a given pulse width.

        :param pulse_width: The pulse width.
        """
        angle = self.pulse_width_to_angle(pulse_width)
        self.rotate(angle)

    def rotate_by_angle(self, angle: Union[int, float]):
        """
        Rotate by a given angle.

        :param angle: The angle.
        """
        self.rotate(angle)

    def rotate(self, angle):
        """
        Rotate the clutch gear by a given angle.

        :param angle: The angle.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('__rotate'))

    def start(self):
        """
        Start the clutch gear emulator.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('start'))

    def stop(self):
        """
        Stop the clutch gear emulator.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('stop'))
