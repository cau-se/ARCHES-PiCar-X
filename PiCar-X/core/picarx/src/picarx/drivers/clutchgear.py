from typing import Union
from picarx.pwm import PWM
from picarx.interfaces.actuators import ClutchGearInterface, SunFounderClutchGear
from abc import abstractmethod


class AbstractClutchGearDriver(ClutchGearInterface):
    """
    Abstract base class for clutch gear drivers.
    """

    def __init__(self, pwm_pin: str, i2c_port: str = '\\dev\\i2c-1'):
        """
        Initialize the clutch gear driver.

        :param pwm_pin: The PWM pin configuration.
        :param i2c_port: The I2C port configuration.
        """
        self.pwm_pin = {'channel': pwm_pin, 'i2c_port': i2c_port}
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
            channel=config['channel'], i2c_port=config['i2c_port'])
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
        if not (isinstance(angle, int) or isinstance(angle, float)):
            raise ValueError(
                "Angle value should be int or float value, not {}".format(type(angle)))

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

    @abstractmethod
    def start(self):
        """
        Start the clutch gear driver.
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


class SimpleClutchGear(AbstractClutchGearDriver):
    """
    Simple implementation of the clutch gear driver. 
    
    This is just for testing purposes. Write a real driver for your clutch gear using the AbstractClutchGearDriver class.
    """

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str]):
        """
        Initialize the simple clutch gear driver.

        :param name: The name of the clutch gear.
        :param direction_pin: The GPIO pin for direction control.
        :param pwm_pin: The PWM pin configuration.
        """
        super(SimpleClutchGear, self).__init__(direction_pin, pwm_pin)
        self.name = name

    @property
    def name(self):
        """
        Get the name of the clutch gear.

        :return: The name of the clutch gear.
        """
        return self.__name

    @name.setter
    def name(self, name: str):
        """
        Set the name of the clutch gear.

        :param name: The name of the clutch gear.
        """
        self.__name = name

    def rotate(self, angle: Union[int, float]):
        """
        Rotate the clutch gear by a given angle.

        :param angle: The angle.
        """
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180

        pulse_width = self.angle_to_pulse_width(angle)
        self.pwm_pin.pulse_width = pulse_width
        self.angle = angle

    def start(self):
        """
        Start the simple clutch gear driver.
        """
        pass

    def stop(self):
        """
        Stop the simple clutch gear driver.
        """
        pass