import unittest
from unittest.mock import patch, MagicMock
from parameterized import parameterized
from picarx.drivers.clutchgear import SimpleClutchGear, SunFounderClutchGear
from picarx.pwm import PWM, SunFounderPWMValues
from picarx.i2c import SMBus


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
