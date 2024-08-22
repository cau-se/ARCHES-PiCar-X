import unittest
from unittest.mock import patch, mock_open, MagicMock, call
import builtins
import os
import weakref
from picarx.gpio import GPIO, Value, Pin, Direction, Edge, Active
from twisted.internet import reactor
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class TestGPIO(unittest.TestCase):

    @patch('builtins.open', new_callable=mock_open)
    @patch.object(reactor, 'callInThread')
    @patch.object(FileSystemEventHandler, 'on_modified')
    @patch('os.path.exists', return_value=True)
    @patch('weakref.finalize', return_value=MagicMock())
    def test_set_value_gpio(self, mock_weakref, mock_os, mock_watchdog, mock_reactor, mock_file):
        pin= GPIO().setup(17, direction=Direction.OUT, callback=None)
        pin.value = Value.HIGH.value
        mock_file.assert_called_with(Pin.SYSFS_GPIO_PIN_VALUE.format(17), 'w')
        mock_file().write.assert_called_with(str(1))
        mock_os.assert_called_once()
        mock_weakref.assert_called_once()

    @patch('builtins.open', new_callable=mock_open)
    @patch.object(reactor, 'callInThread')
    @patch.object(FileSystemEventHandler, 'on_modified')
    @patch('os.path.exists', return_value=True)
    @patch('weakref.finalize', return_value=MagicMock())
    def test_set_value_pin(self, mock_weakref, mock_os, mock_watchdog, mock_reactor, mock_file):
        pin = Pin(17)
        pin.value = 1
        mock_file.assert_called_with(Pin.SYSFS_GPIO_PIN_VALUE.format(17), 'w')
        mock_file().write.assert_called_with(str(1))
        mock_os.assert_called_once()
        mock_weakref.assert_called_once()

    @patch('builtins.open', new_callable=mock_open, read_data="1")
    @patch.object(reactor, 'callInThread')
    @patch.object(FileSystemEventHandler, 'on_modified')
    @patch('os.path.exists', return_value=True)
    @patch('weakref.finalize', return_value=MagicMock())
    def test_read_value_pin(self, mock_weakref, mock_os, mock_watchdog, mock_reactor, mock_file):
        pin = Pin(17)
        self.assertEqual(pin.value, 1)
        mock_file.assert_called_with(Pin.SYSFS_GPIO_PIN_VALUE.format(17), 'r')
        mock_os.assert_called_once()
        mock_weakref.assert_called_once()


if __name__ == '__main__':
    unittest.main()