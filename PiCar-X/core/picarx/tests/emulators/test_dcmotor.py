import unittest
from unittest.mock import patch, MagicMock, mock_open
from picarx.interfaces.actuators import MotorSide, TravelDirection
from picarx.emulators.dcmotor import AbstractMotorEmulator
from watchdog.events import FileSystemEventHandler
from twisted.internet import reactor
from parameterized import parameterized
import io


def to_big_endian(value):
    return int.from_bytes(value.to_bytes(2, 'little'), 'big')

class MotorEmulator(AbstractMotorEmulator):
    """
    Implementation of the motor emulator.
    """

    def __init__(self, *args, **kwargs):
        """
        Initialize the motor emulator.
        """
        super(MotorEmulator, self).__init__(*args, **kwargs)

    def change_direction_listener(self, event):
        """
        Handle direction change events.

        :param event: The event that triggered the direction change.
        """
        if event.event_type == 'modified':
            direction = self.direction_pin.value
            if direction == TravelDirection.FORWARD.value:
                self.drive_forward()
            elif direction == TravelDirection.BACKWARD.value:
                self.drive_backwards()

    def drive_with_speed(self, speed_value):
        """
        Drive the motor with a given speed.

        :param i2c_value: The I2C value representing the speed.
        """
        print("Moving with {} percent speed".format(self.pwm_pin.duty_cycle))

    def start(self):
        """
        Start the motor emulator.
        """
        print('started')

    def stop(self):
        """
        Stop the motor emulator.
        """
        print('stopped')

class TestDCMotor(unittest.TestCase):

    @patch('builtins.open', new_callable=mock_open)
    @patch.object(reactor, 'callInThread')
    @patch.object(FileSystemEventHandler, 'on_modified')
    @patch('os.path.exists', return_value=True)
    @patch('weakref.finalize', return_value=MagicMock())
    @patch('picarx.i2c.SMBus')
    def setUp(self, mock_smbus, mock_finalize, mock_exists,  mock_on_modified,  mock_callInThread, mock_open):
        self.mock_smbus = mock_smbus.return_value
        self.dcmotor = MotorEmulator(
            name="TestMotor",
            direction_pin=24,
            pwm_pin='P12',
            i2c_port='/dev/i2c-1',
            motor_side=MotorSide.LEFT
        )

    def test_initialization(self):
        self.assertEqual(self.dcmotor.name, "TestMotor")
        self.assertEqual(self.dcmotor.direction_pin.pin_number, 24)
        self.assertEqual(
            self.dcmotor.pwm_pin.channel, int('P12'[1:]))
        self.assertEqual(self.dcmotor.pwm_pin.i2c_port, '/dev/i2c-1')
        self.assertEqual(self.dcmotor.motor_side, MotorSide.LEFT)
        self.assertEqual(self.dcmotor.direction, 1)

    @parameterized.expand([
        (65039, "Moving with {} percent speed\n".format(100)),
        (0, "Moving with {} percent speed\n".format(0)),
        (26114, "Moving with {} percent speed\n".format(15)),
    ])
    @patch('sys.stdout', new_callable=io.StringIO)
    def test_set_speed(self, i2c_value, expected_output, mock_stdout):
        self.mock_smbus.read_word_data.return_value = i2c_value
        self.dcmotor.drive_with_speed(self.dcmotor.pwm_pin.duty_cycle)
        self.assertEqual(mock_stdout.getvalue(), expected_output)

    @patch('sys.stdout', new_callable=io.StringIO)
    def test_start(self, mock_stdout):
        self.dcmotor.start()
        self.assertEqual(mock_stdout.getvalue(), 'started\n')
        
    @patch('sys.stdout', new_callable=io.StringIO)
    def test_stop(self, mock_stdout):
        self.dcmotor.stop()
        self.assertEqual(mock_stdout.getvalue(), 'stopped\n')



if __name__ == '__main__':
    unittest.main()
