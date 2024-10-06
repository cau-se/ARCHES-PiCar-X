from typing import Union
from picarx.pwm import PWM
from picarx.interfaces.actuators import ClutchGearInterface, SunFounderClutchGear
from abc import abstractmethod


class AbstractClutchGearDriver(ClutchGearInterface):
    """
    Abstract base class for clutch gear drivers.
    """

    def __init__(self, pwm_pin: str, i2c_port: str = '/dev/i2c-1', address=20):
        """
        Initialize the clutch gear driver.

        :param pwm_pin: The PWM pin configuration.
        :param i2c_port: The I2C port configuration.
        """
        self.pwm_pin = {'channel': pwm_pin,
                        'i2c_port': i2c_port, 'address': address}
        self.angle = 90

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
        self.__pwm_pin.period = 4095
        self.__pwm_pin.prescaler = 8

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

        The clutchgear can only handles angles between 0 and 180 degress.

        :param angle: The angle.
        :raises ValueError: If the angle is not an int or float.
        """

        if angle < 0:
            angle = 0
        elif 180 < angle:
            angle = 180

        self.__angle = angle

    def angle_to_pulse_width(self, angle):
        """
        Convert an angle to a pulse width. 

        The pulse width is calculated using the formula: pulse_width = angle / 180 * (maximum_pulse - minimum_pulse) + minimum_pulse.

        :param angle: The angle.
        :return: The pulse width.
        """
        pulse_width = round(angle / 180 * (SunFounderClutchGear.MAXIMUM_PULSE.value -
                            SunFounderClutchGear.MINIMUM_PULSE.value) + SunFounderClutchGear.MINIMUM_PULSE.value)
        # I don't know why it works, but I haven't found another way
        pulse_width = pulse_width / \
            SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value * self.pwm_pin.period
        return round(pulse_width / 2)

    def pulse_width_to_angle(self, pulse_width):
        """
        Convert a pulse width to an angle. 

        The angle is calculated using the formula: angle = (pulse_width - minimum_pulse) / (maximum_pulse - minimum_pulse) * 180.

        :param pulse_width: The pulse width.
        :return: The angle.
        """
        pulse_width = round(
            pulse_width * 2 * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value / self.pwm_pin.period)
        angle = (pulse_width - SunFounderClutchGear.MINIMUM_PULSE.value) * 180 / \
            (SunFounderClutchGear.MAXIMUM_PULSE.value -
             SunFounderClutchGear.MINIMUM_PULSE.value)
        return round(angle)

    def rotate_by_pulse_width(self, pulse_width: int) -> None:
        """
        Rotate the clutch gear by a given pulse width.

        :param pulse_width: The pulse width.
        """
        self.pwm_pin.pulse_width = pulse_width

    @abstractmethod
    def start(self):
        """
        Starts the clutch gear driver. Use some kind of thread to keep the clutch gear running, e.g. with twisted or asyncio.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('start'))

    @abstractmethod
    def stop(self):
        """
        Stop the clutch gear driver.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('stop'))

    @abstractmethod
    def rotate(self, angle):
        """
        Rotate the clutch gear by a given angle.

        :param angle: The angle.
        """
        raise NotImplementedError(
            "The method {} is not implemented.".format('rotate'))
