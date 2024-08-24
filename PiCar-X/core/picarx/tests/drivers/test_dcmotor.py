import unittest
from unittest.mock import patch, MagicMock, mock_open
from picarx.interfaces.actuators import MotorSide
from picarx.drivers.dcmotor import DCMotor
from watchdog.events import FileSystemEventHandler
from twisted.internet import reactor
from parameterized import parameterized
from picarx.pwm import SunFounderPWMValues


class TestDCMotor(unittest.TestCase):

    @patch('builtins.open', new_callable=mock_open)
    @patch.object(reactor, 'callInThread')
    @patch.object(FileSystemEventHandler, 'on_modified')
    @patch('os.path.exists', return_value=True)
    @patch('weakref.finalize', return_value=MagicMock())
    @patch('picarx.i2c.SMBus')
    def setUp(self, mock_smbus, mock_open, mock_callInThread, mock_on_modified, mock_exists, mock_finalize):
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
        self.mock_smbus.write_word_data.assert_called_with(SunFounderPWMValues.DEFAULT_I2C_ADRESS.value,  SunFounderPWMValues.REGISTER_CHANNEL.value + int('P12'[1:]), expected_pulse_width)

    def test_stop(self):
        self.dc_motor.stop()
        assert self.dc_motor.speed == 0


if __name__ == '__main__':
    unittest.main()
