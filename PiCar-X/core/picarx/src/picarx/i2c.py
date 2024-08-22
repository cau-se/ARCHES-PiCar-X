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

# from .basic import _Basic_class
from abc import ABCMeta, abstractmethod
from smbus2 import SMBus


class I2C(object):
    """
    Represents an I2C interface.

    Attributes:
        i2c_port (int): The I2C port number.
        address (int): The I2C address.
        smbus (SMBus): The SMBus instance.
        known_registers (dict): Dictionary of known registers.

    Methods:
        write_byte(data): Writes a byte to the I2C device.
        read_byte(): Reads a byte from the I2C device.
        create_register(register_address, size): Creates a register of the specified size.
    """

    def __init__(self, i2c_port=1, address=0x14):
        """
        Initializes the I2C interface.

        Args:
            i2c_port (int): The I2C port number. Default is 1.
            address (int): The I2C address. Default is 0x14.
        """
        self.i2c_port = i2c_port  # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        self.smbus = SMBus(self.i2c_port)
        self.address = address
        self.known_registers = {}

    @property
    def i2c_port(self):
        """Gets the I2C port number."""
        return self.__i2c_port

    @i2c_port.setter
    def i2c_port(self, port: int):
        """Sets the I2C port number."""
        self.__i2c_port = port

    @property
    def address(self):
        """Gets the I2C address."""
        return self.__address

    @address.setter
    def address(self, address: bytes):
        """Sets the I2C address."""
        self.__address = address

    def write_byte(self, data):
        """
        Writes a byte to the I2C device.

        Args:
            data (int): The byte to write.

        Returns:
            None
        """
        return self.smbus.write_byte(self.address, data)

    def read_byte(self):
        """
        Reads a byte from the I2C device.

        Returns:
            int: The byte read from the device.
        """
        return self.smbus.read_byte(self.address)

    def create_register(self, register_address, size):
        """
        Creates a register of the specified size.

        Args:
            register_address (int): The address of the register.
            size (int): The size of the register (1, 2, or 4 bytes).

        Returns:
            Register: The created register object.

        Raises:
            ValueError: If the size is not 1, 2, or 4.
        """
        if size == 1:
            return ByteRegister(self.smbus, self.address, register_address)
        elif size == 2:
            return WordRegister(self.smbus, self.address, register_address)
        elif size == 4:
            return I2CBlockRegister(self.smbus, self.address, register_address)
        else:
            raise ValueError(
                "The size of the register must be 1,2, or 4. You entered {}".format(size))


class Register(metaclass=ABCMeta):
    """
    Abstract base class for an I2C register.

    Attributes:
        smbus (SMBus): The SMBus instance.
        address (int): The I2C address.
        register_address (int): The register address.

    Methods:
        write(value): Writes a value to the register.
        read(): Reads a value from the register.
        clean_up(): Cleans up the register by writing 0.
    """

    @abstractmethod
    def __init__(self, smbus, address, register_address):
        """
        Initializes the register.

        Args:
            smbus (SMBus): The SMBus instance.
            address (int): The I2C address.
            register_address (int): The register address.
        """
        self.smbus = smbus
        self.address = address
        self.register_address = register_address

    @abstractmethod
    def write(self, value):
        """Writes a value to the register."""
        pass

    @abstractmethod
    def read(self):
        """Reads a value from the register."""
        pass

    def __del__(self):
        """Destructor to clean up the register."""
        self.clean_up()

    def clean_up(self):
        """Cleans up the register by writing 0."""
        print("Cleaning register: {}".format(self.register_address))
        self.write(0)


class ByteRegister(Register):
    """
    Represents a byte-sized I2C register.

    Methods:
        write(value): Writes a byte to the register.
        read(): Reads a byte from the register.
    """

    def __init__(self, smbus, address, register_address):
        """
        Initializes the byte register.

        Args:
            smbus (SMBus): The SMBus instance.
            address (int): The I2C address.
            register_address (int): The register address.
        """
        super(ByteRegister, self).__init__(smbus, address, register_address)

    def write(self, value):
        """
        Writes a byte to the register.

        Args:
            value (int): The byte to write.

        Returns:
            None
        """
        return self.smbus.write_byte_data(self.address, self.register_address, value)

    def read(self):
        """
        Reads a byte from the register.

        Returns:
            int: The byte read from the register.
        """
        return self.smbus.read_byte_data(self.address, self.register_address)


class WordRegister(Register):
    """
    Represents a word-sized (2 bytes) I2C register.

    Methods:
        write(value): Writes a word to the register.
        read(): Reads a word from the register.
    """

    def __init__(self, smbus, address, register_address):
        """
        Initializes the word register.

        Args:
            smbus (SMBus): The SMBus instance.
            address (int): The I2C address.
            register_address (int): The register address.
        """
        super(WordRegister, self).__init__(smbus, address, register_address)

    def write(self, value):
        """
        Writes a word to the register.

        Args:
            value (int): The word to write.

        Returns:
            None
        """
        value_bytes = int.from_bytes(value.to_bytes(2, 'little'), 'big')
        return self.smbus.write_word_data(self.address, self.register_address, value_bytes)

    def read(self):
        """
        Reads a word from the register.

        Returns:
            int: The word read from the register.
        """
        read_value = self.smbus.read_word_data(
            self.address, self.register_address)
        return int.from_bytes(read_value.to_bytes(2, 'big'), 'little')
