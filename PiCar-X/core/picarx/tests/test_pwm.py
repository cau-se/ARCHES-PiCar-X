# test_pwm.py
import unittest
from unittest.mock import patch
from picarx.pwm import PWM, SunFounderPWMValues
from picarx.i2c import WordRegister

def to_big_endian(value):
    return int.from_bytes(value.to_bytes(2, 'little'), 'big')

class TestPWM(unittest.TestCase):

    @patch('picarx.i2c.SMBus')
    def setUp(self, mock_smbus):
        self.mock_smbus_instance = mock_smbus.return_value
        self.pwm = PWM(channel=1)

    def test_constructor(self):
        self.assertEqual(self.pwm.channel, 1)
        self.assertEqual(self.pwm.debug, "critical")
        self.assertIsInstance(self.pwm.register_channel, WordRegister)
        self.assertIsInstance(self.pwm.register_prescaler, WordRegister)
        self.assertIsInstance(self.pwm.register_frequency, WordRegister)

    def test_channel_property(self):
        self.pwm.channel = "P2"
        self.assertEqual(self.pwm.channel, 2)
        with self.assertRaises(ValueError):
            self.pwm.channel = "X2"

    def test_period_property(self):
        self.pwm.period = 1000
        self.assertEqual(self.pwm.period, 999)

    def test_prescaler_property(self):
        self.pwm.prescaler = 10
        expect_result = to_big_endian(10)
        self.mock_smbus_instance.write_word_data.assert_called_with(
            SunFounderPWMValues.DEFAULT_I2C_ADRESS.value, SunFounderPWMValues.REGISTER_PRESCALER.value + 1, expect_result
        )

    def test_duty_cycle_property(self):
        self.pwm.duty_cycle = 50
        expect_result = to_big_endian(int(50 / 100.0 * 4095 ))
        self.mock_smbus_instance.write_word_data.assert_called_with(SunFounderPWMValues.DEFAULT_I2C_ADRESS.value,
            SunFounderPWMValues.REGISTER_CHANNEL.value + 1, expect_result
        )

    def test_frequency_property(self):
        self.pwm.frequency = 1000
        expect_result = to_big_endian(1000)
        self.mock_smbus_instance.write_word_data.assert_called_with(
            SunFounderPWMValues.DEFAULT_I2C_ADRESS.value, SunFounderPWMValues.REGISTER_FREQUENCY.value + 1, expect_result
        )

if __name__ == '__main__':
    unittest.main()