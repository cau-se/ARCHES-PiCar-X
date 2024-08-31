from typing import Union
import unittest
from unittest.mock import patch
from parameterized import parameterized
from picarx.drivers.clutchgear import SunFounderClutchGear, AbstractClutchGearDriver
from picarx.pwm import  SunFounderPWMValues

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


def angle_to_pulse_width(angle, period):
    pulse_width = round(angle / 180 * (SunFounderClutchGear.MAXIMUM_PULSE.value -
                        SunFounderClutchGear.MINIMUM_PULSE.value) + SunFounderClutchGear.MINIMUM_PULSE.value)
    # I don't know why it works, but I haven't found another way
    pulse_width = pulse_width / \
        SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value * period
    return round(pulse_width / 2)


def pulse_width_to_angle(pulse_width, period):
    pulse_width = round(
        pulse_width * 2 * SunFounderClutchGear.SUNFOUNDER_RANDOM_DIVIDER.value / period)
    angle = (pulse_width - SunFounderClutchGear.MINIMUM_PULSE.value) * 180 / \
        (SunFounderClutchGear.MAXIMUM_PULSE.value -
         SunFounderClutchGear.MINIMUM_PULSE.value)
    return round(angle)


def to_big_endian(value):
    return int.from_bytes(value.to_bytes(2, 'little'), 'big')


def from_big_endian(read_value):
    return int.from_bytes(read_value.to_bytes(2, 'big'), 'little')


class TestSimpleClutchGear(unittest.TestCase):

    @patch('picarx.i2c.SMBus')
    def setUp(self, mock_pwm):
        self.mock_pwm_instance = mock_pwm.return_value
        self.clutch_gear = SimpleClutchGear(
            name="TestGear", direction_pin=1, pwm_pin="P1")

    @parameterized.expand([
        (90, 154),
        (0, 51),
        (180, 256),
    ])
    def test_angle_to_pulse_width(self, angle, expected_pulse_width):
        pulse_width = self.clutch_gear.angle_to_pulse_width(angle)
        self.assertEqual(pulse_width, expected_pulse_width)

    @parameterized.expand([
        (51, 0),
        (154, 90),
        (256, 180)
    ])
    def test_pulse_width_to_angle(self, pulse_width, expected_angle):
        angle = self.clutch_gear.pulse_width_to_angle(pulse_width)
        self.assertEqual(angle, expected_angle)

    @parameterized.expand([
        (0, 51),
        (90, 154),
        (180, 256),
        (270, 256)
    ])
    def test_rotate(self, angle, expected_pulse_width):
        self.clutch_gear.rotate(angle)
        self.mock_pwm_instance.write_word_data.assert_called_with(
            SunFounderPWMValues.DEFAULT_I2C_ADRESS.value, SunFounderPWMValues.REGISTER_CHANNEL.value + 1, to_big_endian(expected_pulse_width)
        )


if __name__ == '__main__':
    unittest.main()
