import unittest
from unittest.mock import patch, MagicMock, mock_open
from twisted.internet import reactor
from parameterized import parameterized
from picarx.emulators.clutchgear import AbstractClutchGearEmulator
import io

class SimpleClutchGearEmulator(AbstractClutchGearEmulator):
    """
    Simple implementation of the clutch gear emulator.
    
    This is just for testing purposes. Write a real emulator for your clutch gear using the AbstractClutchGearEmulator class.
    """

    def __init__(self, pwm_pin, frequency=50):
        """
        Initialize the simple clutch gear emulator.

        :param pwm_pin: The PWM pin configuration.
        :param frequency: The frequency in Hertz.
        """
        super(SimpleClutchGearEmulator, self).__init__(pwm_pin, frequency)

    def rotate(self, angle):
        """
        Rotate the clutch gear by a given angle.

        :param angle: The angle.
        """
        print("Rotating by angle: {}".format(angle))

    def read_i2c_value(self):
        """
        Read the I2C value and rotate by the corresponding pulse width.
        """
        self.rotate_by_pulse_width(self.pwm_pin.pulse_width)

    def start(self):
        """
        Start the simple clutch gear emulator.
        """
        print('started')

    def stop(self):
        """
        Stop the simple clutch gear emulator.
        """
        print('stopped')

class TestSimpleClutchGearEmulator(unittest.TestCase):

    @patch('builtins.open', new_callable=mock_open)
    @patch.object(reactor, 'callInThread')
    @patch('os.path.exists', return_value=True)
    @patch('weakref.finalize', return_value=MagicMock())
    @patch('picarx.i2c.SMBus')
    def setUp(self, mock_smbus, mock_finalize, mock_exists,  mock_on_modified,  mock_callInThread):
        self.mock_smbus = mock_smbus.return_value
        self.clutch_gear_emulator = SimpleClutchGearEmulator(
            pwm_pin="P1",
        )

    def test_initialization(self):
        self.assertEqual(self.clutch_gear_emulator.pwm_pin.channel, 1)
        self.assertEqual(self.clutch_gear_emulator.frequency, 1/50)

    @parameterized.expand([
        (90, 154),
        (0, 51),
        (180, 256),
    ])
    def test_angle_to_pulse_width(self, angle, expected_pulse_width):
        pulse_width = self.clutch_gear_emulator.angle_to_pulse_width(angle)
        self.assertEqual(pulse_width, expected_pulse_width)

    @parameterized.expand([
        (51, 0),
        (154, 90),
        (256, 180)
    ])
    def test_pulse_width_to_angle(self, pulse_width, expected_angle):
        angle = self.clutch_gear_emulator.pulse_width_to_angle(pulse_width)
        self.assertEqual(angle, expected_angle)

    @patch('sys.stdout', new_callable=io.StringIO)
    def test_rotate(self, mock_stdout):
        self.clutch_gear_emulator.rotate(90)
        self.assertEqual(mock_stdout.getvalue(), "Rotating by angle: 90\n")

    @patch('sys.stdout', new_callable=io.StringIO)
    def test_start(self, mock_stdout):
        self.clutch_gear_emulator.start()
        self.assertEqual(mock_stdout.getvalue(), "started\n")
        
    @patch('sys.stdout', new_callable=io.StringIO)
    def test_stop(self, mock_stdout):
        self.clutch_gear_emulator.stop()
        self.assertEqual(mock_stdout.getvalue(), "stopped\n")


if __name__ == '__main__':
    unittest.main()