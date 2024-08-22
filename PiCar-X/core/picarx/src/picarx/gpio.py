#!/usr/bin/env python3
# Copyright 2022 - 2024 Alexander Barbie
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum, EnumMeta
from typing import Union
from twisted.internet import reactor
import os
import weakref
import time
import logging
from watchdog.events import FileSystemEventHandler
from datetime import datetime, timedelta
from watchdog.observers import Observer


BUFFER_LEN = 100


class GPIOEnumMeta(EnumMeta):
    def __contains__(cls, item):
        return item in [v.value for v in cls.__members__.values()]


class Active(Enum, metaclass=GPIOEnumMeta):
    ON = 1
    OFF = 0


class Value(Enum, metaclass=GPIOEnumMeta):
    HIGH = 1
    LOW = 0


class Direction(Enum, metaclass=GPIOEnumMeta):
    OUT = 'out'
    IN = 'in'


class Edge(Enum, metaclass=GPIOEnumMeta):
    RISING = 'rising'
    FALLING = 'falling'
    NONE = 'none'
    BOTH = 'both'


class Pin(FileSystemEventHandler):
    """
    Represents a GPIO pin and provides methods to interact with it.

    Attributes:
        SYSFS_BASE_PATH (str): Base path for sysfs GPIO.
        SYSFS_GPIO_EXPORT (str): Path to export GPIO.
        SYSFS_GPIO_UNEXPORT (str): Path to unexport GPIO.
        SYSFS_GPIO_PIN_PATH (str): Path to GPIO pin.
        SYSFS_GPIO_PIN_DIRECTION (str): Path to GPIO pin direction.
        SYSFS_GPIO_PIN_VALUE (str): Path to GPIO pin value.
        SYSFS_GPIO_PIN_EDGE (str): Path to GPIO pin edge.
        SYSFS_GPIO_PIN_ACTIVE_LOW (str): Path to GPIO pin active low.

    Methods:
        __init__(pin_number, direction, edge, active, callback, emulator): Initializes the GPIO pin.
        on_modified(event): Callback for when the pin value is modified.
        __read_path(path): Reads the value from the given path.
        on(): Sets the pin value to high.
        off(): Sets the pin value to low.
        high(): Alias for on().
        low(): Alias for off().
        start_watchdog(): Starts the watchdog to monitor pin changes.
        exists_already(pin_number): Checks if the pin already exists.
    """

    SYSFS_BASE_PATH = '/sys/class/gpio'
    SYSFS_GPIO_EXPORT = '/sys/class/gpio/export'
    SYSFS_GPIO_UNEXPORT = '/sys/class/gpio/unexport'
    SYSFS_GPIO_PIN_PATH = '/sys/class/gpio/gpio{}'
    SYSFS_GPIO_PIN_DIRECTION = '/sys/class/gpio/gpio{}/direction'
    SYSFS_GPIO_PIN_VALUE = '/sys/class/gpio/gpio{}/value'
    SYSFS_GPIO_PIN_EDGE = '/sys/class/gpio/gpio{}/edge'
    SYSFS_GPIO_PIN_ACTIVE_LOW = '/sys/class/gpio/gpio{}/active_low'

    _dict = {
        "BOARD_TYPE": 12,
    }

    GPIOPins = {
        "D0":  17,
        "D1":  18,
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25,
        "D7":  4,
        "D8":  5,
        "D9":  6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  19,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 21,
    }

    def __init__(self, pin_number: Union[int, str], direction=Direction.OUT, edge=Edge.NONE, active=Active.ON, callback=None, emulator=False):
        """
        Initializes the GPIO pin.

        Args:
            pin_number (Union[int, str]): The pin number or name.
            direction (Direction): The direction of the pin (IN or OUT).
            edge (Edge): The edge detection setting.
            active (Active): The active state of the pin.
            callback (callable): The callback function to be called on pin value change.
            emulator (bool): Whether to run in emulator mode.
        """
        self.__emulator = emulator
        self.pin_number = pin_number

        while not os.path.exists(Pin.SYSFS_GPIO_PIN_PATH.format(self.pin_number)):
            time.sleep(0.1)

        if not emulator:
            self.direction = direction
            self.edge = edge
            self.active_low = active
        else:
            self.__init_existing()

        self.value = None
        self.callback = callback
        self.last_modified = datetime.now()
        self._finalize = weakref.finalize(
            self, self.__unexport, self.pin_number)

    def __init_existing(self):
        """
        Initializes the pin for emulator mode.
        """
        logging.info("Reading Pin for Emulator")
        time.sleep(2)

    def on_modified(self, event):
        """
        Callback for when the pin value is modified.

        Args:
            event: The event object.
        """
        if datetime.now() - self.last_modified < timedelta(seconds=1):
            return
        else:
            self.last_modified = datetime.now()

        if callable(self.callback):
            self.callback(event)

    def __read_path(self, path):
        """
        Reads the value from the given path.

        Args:
            path (str): The path to read from.

        Returns:
            str: The value read from the path.
        """
        if os.path.isdir(path):
            path_reader = open(path, 'r')
            value = path_reader.read()
            path_reader.seek(0)
            path_reader.close()
            return value
        else:
            raise ValueError("The path {} does not exist".format(path))

    @property
    def pin_number(self):
        """Returns the pin number."""
        return self.__pin_number

    @pin_number.setter
    def pin_number(self, pin_number: Union[int, str]):
        """Sets the pin number."""
        if isinstance(pin_number, str):
            if pin_number in self.GPIOPins.keys():
                self.__pin_number = self.GPIOPins[pin_number]
        else:
            self.__pin_number = pin_number

        if not self.__emulator:
            self.__export()

    @property
    def direction(self):
        """Returns the direction of the pin."""
        return self.__direction

    @direction.setter
    def direction(self, direction):
        """Sets the direction of the pin."""
        if direction.value in Direction:
            if not self.__emulator:
                with open(Pin.SYSFS_GPIO_PIN_DIRECTION.format(self.pin_number), "w") as pin:
                    pin.write(direction.value)
            self.__direction = Direction(direction)

    @property
    def edge(self):
        """Returns the edge detection setting of the pin."""
        return self.__edge

    @edge.setter
    def edge(self, edge):
        """Sets the edge detection setting of the pin."""
        if edge in Edge:
            if not self.__emulator:
                with open(Pin.SYSFS_GPIO_PIN_EDGE.format(self.pin_number), "w") as pin:
                    pin.write(edge)
            self.__edge = edge

    @property
    def value(self):
        """Returns the value of the pin."""
        with open(Pin.SYSFS_GPIO_PIN_VALUE.format(self.pin_number), 'r') as value_reader:
            value = value_reader.read()
            if value != '':
                value = int(value)
            return value

    @value.setter
    def value(self, val):
        """Sets the value of the pin."""
        if val is not None:
            if val in Value:
                with open(Pin.SYSFS_GPIO_PIN_VALUE.format(self.pin_number), 'w') as value_writer:
                    value_writer.write("{}".format(val))

    @property
    def callback(self):
        """Returns the callback function."""
        return self.__callback

    @callback.setter
    def callback(self, callback):
        """Sets the callback function."""
        if callable(callback) or callback is None:
            self.__callback = callback
        else:
            raise TypeError("Your callback is not callable.")

    @property
    def active_low(self):
        """Returns the active low setting of the pin."""
        return self.__active_low

    @active_low.setter
    def active_low(self, active_low_mode):
        """Sets the active low setting of the pin."""
        if active_low_mode in Active:
            if not self.__emulator:
                with open(Pin.SYSFS_GPIO_PIN_ACTIVE_LOW.format(self.pin_number), "w") as pin:
                    pin.write(active_low_mode)
            self.__active_low = active_low_mode

    @property
    def value_reader(self):
        """Returns the value reader."""
        return self.__value_reader

    @value_reader.setter
    def value_reader(self, path):
        """Sets the value reader."""
        if path is None:
            if hasattr(self, 'value_reader'):
                self.__value_reader.close()
            self.__value_reader = None
        else:
            self.__value_reader = open(path, "r+")

    def on(self):
        """Sets the pin value to high."""
        self.value = 1

    def off(self):
        """Sets the pin value to low."""
        self.value = 0

    def high(self):
        """Alias for on()."""
        self.on()

    def low(self):
        """Alias for off()."""
        self.off()

    def __export(self):
        """Exports the pin."""
        if not self.__emulator:
            with open(Pin.SYSFS_GPIO_EXPORT, "w") as export:
                export.write('%d' % self.pin_number)
            logging.debug("Exported GPIO{}".format(self.pin_number))
        else:
            logging.debug(
                "Did not export the pin {} since I am an emulator".format(self.pin_number))

    def __unexport(self, *args):
        """Unexports the pin."""
        if not self.__emulator:
            with open(Pin.SYSFS_GPIO_UNEXPORT, "w") as unexport:
                unexport.write('%d' % self.pin_number)
            logging.debug("Unexported GPIO{}".format(self.pin_number))
        else:
            logging.debug(
                "I am an emulator, thus I will not unexport pin {}".format(self.pin_number))

    def start_watchdog(self):
        """
        Starts the watchdog to monitor pin changes.
        """
        if callable(self.callback):
            observer = Observer()
            observer.schedule(self, self.SYSFS_GPIO_PIN_VALUE.format(
                self.pin_number), recursive=False)
            observer.start()
            self.running = True
            try:
                while reactor.running:
                    time.sleep(1/50)
            finally:
                observer.stop()
            observer.join()
        else:
            raise TypeError("Callback is not defined.")

    @staticmethod
    def exists_already(pin_number):
        """
        Checks if the pin already exists.

        Args:
            pin_number (int): The pin number.

        Returns:
            bool: True if the pin exists, False otherwise.
        """
        gpio_path = Pin.SYSFS_GPIO_PIN_PATH.format(pin_number)
        return os.path.isdir(gpio_path)


class GPIO(object):
    """
    Represents the GPIO interface and provides methods to interact with GPIO pins.

    Methods:
        stop(): Stops the GPIO interface.
        cleanup(): Cleans up the GPIO interface.
        setmode(mode): Sets the GPIO mode.
        getmode(): Gets the GPIO mode.
        setup(pin_number, direction, callback, emulator): Sets up a GPIO pin.
    """

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, '_instance'):
            instance = super(GPIO, cls).__new__(cls)
            instance.__pins = {}
            instance._running = True
            reactor.addSystemEventTrigger('before', 'shutdown', instance.stop)
            cls._instance = instance
        return cls._instance

    def stop(cls):
        """Stops the GPIO interface."""
        cls._running = False
        cls.cleanup()

    def cleanup(cls):
        """Cleans up the GPIO interface."""
        cls.__pins.clear()

    def setmode(cls, mode):
        """Sets the GPIO mode."""
        cls.__mode = mode

    def getmode(cls):
        """Gets the GPIO mode."""
        return cls.__mode

    def setup(cls, pin_number, direction=Direction.OUT, callback=None, emulator=False):
        """
        Sets up a GPIO pin.

        Args:
            pin_number (Union[int, str]): The pin number or name.
            direction (Direction): The direction of the pin (IN or OUT).
            callback (callable): The callback function to be called on pin value change.
            emulator (bool): Whether to run in emulator mode.

        Returns:
            Pin: The initialized pin object.
        """
        if not emulator:
            new_pin = Pin(pin_number, direction=direction, callback=callback)
        else:
            new_pin = Pin(pin_number, callback=callback, emulator=True)

        cls.__pins.update({pin_number: new_pin})
        reactor.callInThread(new_pin.start_watchdog)
        return new_pin
