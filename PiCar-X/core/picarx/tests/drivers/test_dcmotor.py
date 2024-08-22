import pytest
import unittest
from unittest.mock import patch, MagicMock, mock_open
from picarx.interfaces.actuators import MotorSide, TravelDirection
from picarx.gpio import GPIO
from picarx.pwm import PWM
from picarx.drivers.dcmotor import DCMotor
from watchdog.events import FileSystemEventHandler
from twisted.internet import reactor
from parameterized import parameterized


class TestDCMotor(unittest.TestCase):

    @patch('picarx.i2c.SMBus')
    @patch('builtins.open', new_callable=mock_open)
    @patch.object(reactor, 'callInThread')
    @patch.object(FileSystemEventHandler, 'on_modified')
    @patch('os.path.exists', return_value=True)
    @patch('weakref.finalize', return_value=MagicMock())
    def setUp(self, mock_smbus, mock_open, mock_callInThread, mock_on_modified, mock_exists, mock_finalize):
        self.mock_smbus = mock_smbus.return_value
        self.dc_motor = DCMotor(
            name="TestMotor", direction_pin=17, pwm_pin=18, motor_side=MotorSide.LEFT)

    @parameterized.expand([
        (100, 4094),
        (0, 0),
        (101, 4094),
        (13, 614),
        (15, 614),
    ])
    def test_drive_with_speed(self, speed, expected_pulse_width):
        self.dc_motor.drive_with_speed(speed)
        assert self.dc_motor.pwm_pin.pulse_width.assert_called_with(expected_pulse_width)

    def test_start(self):
        self.dc_motor.start()
        # Hier können spezifische Assertions hinzugefügt werden, wenn die start-Methode implementiert ist

    def test_stop(self):
        self.dc_motor.stop()
        assert self.dc_motor.speed == 0


if __name__ == '__main__':
    unittest.main()
