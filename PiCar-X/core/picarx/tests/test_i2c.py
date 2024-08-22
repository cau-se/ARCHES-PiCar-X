import unittest
from unittest.mock import MagicMock, patch
from picarx.i2c import I2C, ByteRegister, WordRegister

class TestI2C(unittest.TestCase):
    @patch('picarx.i2c.SMBus')
    def setUp(self, mock_smbus):
        self.mock_smbus = mock_smbus
        self.i2c = I2C(i2c_port=1, address=0x14)

    def test_write_byte(self):
        self.i2c.write_byte(0xFF)
        self.mock_smbus.return_value.write_byte.assert_called_once_with(0x14, 0xFF)

    def test_read_byte(self):
        self.mock_smbus.return_value.read_byte.return_value = 0xFF
        result = self.i2c.read_byte()
        self.mock_smbus.return_value.read_byte.assert_called_once_with(0x14)
        self.assertEqual(result, 0xFF)

class TestByteRegister(unittest.TestCase):
    @patch('picarx.i2c.SMBus')
    def test_write_and_read(self, mock_smbus):
        byte_register = ByteRegister(mock_smbus, 0x14, 0x01)
        byte_register.write(0xFF)
        mock_smbus.write_byte_data.assert_called_once_with(0x14, 0x01, 0xFF)
        mock_smbus.read_byte_data.return_value = 0xFF
        result = byte_register.read()
        mock_smbus.read_byte_data.assert_called_once_with(0x14, 0x01)
        self.assertEqual(result, 0xFF)

class TestWordRegister(unittest.TestCase):
    @patch('picarx.i2c.SMBus')
    def test_write_and_read(self, mock_smbus):
        word_register = WordRegister(mock_smbus, 0x14, 0x02)
        word_register.write(0x0102)
        # Beachten Sie die Umwandlung von Little-Endian zu Big-Endian und zurück
        mock_smbus.write_word_data.assert_called_once_with(0x14, 0x02, 0x0201)
        mock_smbus.read_word_data.return_value = 0x0201
        result = word_register.read()
        self.assertEqual(result, 0x0102)

if __name__ == '__main__':
    unittest.main()