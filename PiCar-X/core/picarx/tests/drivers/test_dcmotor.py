from typing import Union
import unittest
from unittest.mock import patch, MagicMock, mock_open
from picarx.interfaces.actuators import MotorSide
from picarx.drivers.dcmotor import AbstractDCMotorDriver
from watchdog.events import FileSystemEventHandler
from twisted.internet import reactor
from parameterized import parameterized
from picarx.pwm import SunFounderPWMValues

class DCMotor(AbstractDCMotorDriver):
    """
    Implementation of the DC motor driver.
    This is just for testing purposes. Write a real driver for your DCMotor using the AbstractDCMotorDriver class.

    """

    def __init__(self, name: str, direction_pin: Union[int, str], pwm_pin: Union[int, str], motor_side: MotorSide = MotorSide.LEFT):
        """
        Initialize the DC motor driver.

        :param name: The name of the motor.
        :param direction_pin: The GPIO pin for direction control.
        :param pwm_pin: The PWM pin configuration.
        :param motor_side: The side of the motor (left or right).
        """
        super(DCMotor, self).__init__(name, direction_pin, pwm_pin, motor_side)

    def start(self):
        """
        Start the motor.
        """
        pass

    def stop(self):
        """
        Stop the motor.
        """
        self.drive_with_speed(0)
        
    def drive_with_speed(self, speed: int):
        self.speed = speed

class TestDCMotor(unittest.TestCase):

    @patch('builtins.open', new_callable=mock_open)
    @patch.object(reactor, 'callInThread')
    @patch.object(FileSystemEventHandler, 'on_modified')
    @patch('os.path.exists', return_value=True)
    @patch('weakref.finalize', return_value=MagicMock())
    @patch('picarx.i2c.SMBus')
    def setUp(self, mock_smbus, mock_finalize, mock_exists,  mock_on_modified,  mock_callInThread, mock_open):
        self.mock_smbus = mock_smbus.return_value
        self.dc_motor = DCMotor(
            name="TestMotor", direction_pin=24, pwm_pin='P12', motor_side=MotorSide.LEFT)

    @parameterized.expand([
        (100, 65039),
        (0, 0),
        (101, 65039),
        (13, 26114),
        (15, 26114),
    ])
    def test_drive_with_speed(self, speed, expected_pulse_width):
        self.dc_motor.drive_with_speed(speed)
        self.mock_smbus.write_word_data.assert_called_with(
            SunFounderPWMValues.DEFAULT_I2C_ADRESS.value,  SunFounderPWMValues.REGISTER_CHANNEL.value + int('P12'[1:]), expected_pulse_width)

    def test_stop(self):
        self.dc_motor.stop()
        assert self.dc_motor.speed == 0


if __name__ == '__main__':
    unittest.main()
