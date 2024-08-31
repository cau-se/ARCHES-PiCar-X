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

from typing import Union
from picarx.i2c import I2C
from enum import Enum


class SunFounderPWMValues(Enum):
    """
    Enum class for default PWM values.
    """
    DEFAULT_CLOCK = 72000000
    DEFAULT_PERIOD = 4095
    DEFAULT_PRESCALER = 8
    DEFAULT_I2C_ADRESS = 0x14  # int int: 20
    REGISTER_CHANNEL = 0x20  # in int: 32
    REGISTER_FREQUENCY = 0x30
    REGISTER_PRESCALER = 0x40
    REGISTER_ARR = 0x44
    DEFAULT_FREQUENCY = 50


class PWM(I2C):
    """
    Represents a PWM (Pulse Width Modulation) interface.
    On the Raspberry PI, the PWM is controlled via I2C.

    Attributes:
        channel (Union[str, int]): The PWM channel.
        address (int): The I2C address.
        i2c_port (int): The I2C port number.
        period (int): The PWM period.
        prescaler (int): The PWM prescaler.
        frequency (int): The PWM frequency.
        register_channel (int): The register address for the channel.
        register_frequency (int): The register address for the frequency.
        register_prescaler (int): The register address for the prescaler.
        debug (str): The debug level.

    Methods:
        channel: Gets or sets the PWM channel.
        period: Gets or sets the PWM period.
        prescaler: Gets or sets the PWM prescaler.
        frequency: Gets or sets the PWM frequency.
        register_channel: Gets or sets the register for the channel.
        register_frequency: Gets or sets the register for the frequency.
        register_prescaler: Gets or sets the register for the prescaler.
        pulse_width: Gets or sets the pulse width.
        duty_cycle: Gets or sets the duty cycle.
    """

    def __init__(self, channel, address=0x14, i2c_port=1, period=SunFounderPWMValues.DEFAULT_PERIOD.value, prescaler=SunFounderPWMValues.DEFAULT_PRESCALER.value, frequency=SunFounderPWMValues.DEFAULT_FREQUENCY.value, reg_chn=SunFounderPWMValues.REGISTER_CHANNEL.value, reg_fre=SunFounderPWMValues.REGISTER_FREQUENCY.value, reg_psc=SunFounderPWMValues.REGISTER_PRESCALER.value, reg_arr=SunFounderPWMValues.REGISTER_ARR.value, debug="critical"):
        """
        Initializes the PWM interface.

        Args:
            channel (Union[str, int]): The PWM channel.
            address (int): The I2C address. Default is 0x14.
            i2c_port (int): The I2C port number. Default is 1.
            period (int): The PWM period. Default is 4095.
            prescaler (int): The PWM prescaler. Default is 8.
            frequency (int): The PWM frequency. Default is 50.
            reg_chn (int): The register address for the channel. Default is 0x20.
            reg_fre (int): The register address for the frequency. Default is 0x30.
            reg_psc (int): The register address for the prescaler. Default is 0x40.
            reg_arr (int): The register address for the ARR. Default is 0x44.
            debug (str): The debug level. Default is "critical".
        """
        super(PWM, self).__init__(i2c_port=i2c_port, address=address)
        self.debug = debug
        self.channel = channel
        self.register_channel = reg_chn  # Channel
        self.register_frequency = reg_fre  # Frequency
        self.register_prescaler = reg_psc  # Prescaler
        self.period = 4095
        self.frequency = frequency

    @property
    def channel(self):
        """
        Gets the PWM channel.

        Returns:
            Union[str, int]: The PWM channel.
        """
        return self.__channel

    @channel.setter
    def channel(self, channel: Union[str, int]):
        """
        Sets the PWM channel.

        Args:
            channel (Union[str, int]): The PWM channel.

        Raises:
            ValueError: If the channel is not in the correct format.
            IOError: If there is an error sending data.
        """
        if isinstance(channel, str):
            if channel.startswith("P"):
                channel = int(channel[1:])
            else:
                raise ValueError(
                    "PWM channel should be between [P1, P14], not {0}".format(channel))
        self.__channel = channel

        try:
            self.write_byte(0x2C)
            self.write_byte(0)
            self.write_byte(0)
        except IOError:
            raise IOError("Can't send stuff.")

    @property
    def period(self):
        """
        Gets the PWM period.

        Returns:
            int: The PWM period.
        """
        return self.__period

    @period.setter
    def period(self, period):
        """
        Sets the PWM period.

        Args:
            period (int): The PWM period.
        """
        period = int(period) - 1
        if period > 0:
            self.__period = period

    @property
    def prescaler(self):
        """
        Gets the PWM prescaler.

        Returns:
            int: The PWM prescaler.
        """
        return self.__prescaler

    @prescaler.setter
    def prescaler(self, prescaler):
        """
        Sets the PWM prescaler.

        Args:
            prescaler (int): The PWM prescaler.
        """
        self.__prescaler = prescaler
        if prescaler > 0:
            self.register_prescaler.write(prescaler)

    @property
    def frequency(self):
        """
        Gets the PWM frequency.

        Returns:
            int: The PWM frequency.
        """
        return self.__frequency

    @frequency.setter
    def frequency(self, frequency):
        """
        Sets the PWM frequency.

        Args:
            frequency (int): The PWM frequency.
        """
        self.__frequency = frequency
        if frequency > 0:
            self.register_frequency.write(frequency)

    @property
    def register_channel(self):
        """
        Gets the register for the channel.

        Returns:
            Register: The register for the channel.
        """
        return self.__register_channel

    @register_channel.setter
    def register_channel(self, register_address):
        """
        Sets the register for the channel.

        Args:
            register_address (int): The register address for the channel.
        """
        self.__register_channel = self.create_register(
            register_address + self.channel, 2)

    @property
    def register_frequency(self):
        """
        Gets the register for the frequency.

        Returns:
            Register: The register for the frequency.
        """
        return self.__register_frequency

    @register_frequency.setter
    def register_frequency(self, register_address):
        """
        Sets the register for the frequency.

        Args:
            register_address (int): The register address for the frequency.
        """
        self.__register_frequency = self.create_register(
            register_address + self.channel, 2)

    @property
    def register_prescaler(self):
        """
        Gets the register for the prescaler.

        Returns:
            Register: The register for the prescaler.
        """
        return self.__register_prescaler

    @register_prescaler.setter
    def register_prescaler(self, register_address):
        """
        Sets the register for the prescaler.

        Args:
            register_address (int): The register address for the prescaler.
        """
        self.__register_prescaler = self.create_register(
            register_address + self.channel, 2)

    @property
    def pulse_width(self):
        """
        Gets the pulse width.

        Returns:
            int: The pulse width.
        """
        return self.register_channel.read()

    @pulse_width.setter
    def pulse_width(self, pulse_width: int):
        """
        Sets the pulse width.

        Args:
            pulse_width (int): The pulse width.
        """
        if pulse_width < 0:
            pulse_width = 0
        self.register_channel.write(pulse_width)

    @property
    def duty_cycle(self):
        """
        Gets the duty cycle as percentage (0-100).

        Returns:
            float: The duty cycle as a percentage.
        """
        return round(self.pulse_width/4095 * 100)

    @duty_cycle.setter
    def duty_cycle(self, percentage):
        """
        Sets the duty cycle.

        Args:
            percentage (float): The duty cycle as a percentage.
        """
        if percentage < 0:
            percentage = 0
            
        if percentage > 100:
            percentage = 100
        
        self.pulse_width = int(percentage / 100 * self.period)
