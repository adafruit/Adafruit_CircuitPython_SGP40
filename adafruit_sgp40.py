# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_sgp40`
================================================================================

CircuitPython library for the Adafruit SGP40 Air Quality Sensor / VOC Index Sensor Breakouts


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* Adafruit SGP40 Breakout <https://www.adafruit.com/product/4829>

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases



 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

"""
from time import sleep
from struct import unpack_from, pack_into
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device
from adafruit_register.i2c_struct import ROUnaryStruct, Struct
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_bit import RWBit, ROBit

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_SGP40.git"

_WORD_LEN = 2

class CV:
    """struct helper"""

    @classmethod
    def add_values(cls, value_tuples):
        """Add CV values to the class"""
        cls.string = {}
        cls.lsb = {}

        for value_tuple in value_tuples:
            name, value, string, lsb = value_tuple
            setattr(cls, name, value)
            cls.string[value] = string
            cls.lsb[value] = lsb

    @classmethod
    def is_valid(cls, value):
        """Validate that a given value is a member"""
        return value in cls.string


class Gain(CV):
    """Options for `gain`

    From https://www.tablesgenerator.com/text_tables w/ 'use restructuredText syntax' option
    +---------------------------+----------------------------+
    | Gain                      | Raw Measurement Multiplier |
    +===========================+============================+
    | :py:const:`Gain.GAIN_1X`  | 1                          |
    +---------------------------+----------------------------+
    | :py:const:`Gain.GAIN_3X`  | 3                          |
    +---------------------------+----------------------------+
    | :py:const:`Gain.GAIN_6X`  | 6                          |
    +---------------------------+----------------------------+
    | :py:const:`Gain.GAIN_9X`  | 9                          |
    +---------------------------+----------------------------+
    | :py:const:`Gain.GAIN_18X` | 18                         |
    +---------------------------+----------------------------+

    """


Gain.add_values(
    (
        ("GAIN_1X", 0, "1X", None),
        ("GAIN_3X", 1, "3X", None),
        ("GAIN_6X", 2, "6X", None),
        ("GAIN_9X", 3, "9X", None),
        ("GAIN_18X", 4, "18X", None),
    )
)


class UnalignedStruct(Struct):
    """Class for reading multi-byte data registers with a data length less than the full bitwidth
    of the registers. Most registers of this sort are left aligned to preserve the sign bit"""

    def __init__(self, register_address, struct_format, bitwidth, length):
        super().__init__(register_address, struct_format)
        self._width = bitwidth
        self._num_bytes = length

    def __get__(self, obj, objtype=None):
        # read bytes into buffer at correct alignment
        raw_value = unpack_from(self.format, self.buffer, offset=1)[0]

        with obj.i2c_device as i2c:
            i2c.write_then_readinto(
                self.buffer,
                self.buffer,
                out_start=0,
                out_end=1,
                in_start=2,  # right aligned
                # in_end=4 # right aligned
            )
        raw_value = unpack_from(self.format, self.buffer, offset=1)[0]
        return raw_value >> 8

    def __set__(self, obj, value):
        pack_into(self.format, self.buffer, 1, value)
        with obj.i2c_device as i2c:
            i2c.write(self.buffer)


class SGP40:
    """Class to use the SGP40 Ambient Light and UV sensor"""

    # _reset_bit = RWBit(_CTRL, 4)
    # _gain_bits = RWBits(3, _GAIN, 0)
    # _id_reg = ROUnaryStruct(0xFF, "<B")
    # _uvs_data_reg = UnalignedStruct(_UVSDATA, "<I", 24, 3)  # bits, bytes to read/write
    """Ask the sensor if new data is available"""

    def __init__(self, i2c, address=0x59):
        print("Address:", hex(address), "(%s)"%type(address))

        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        # if self._id_reg != "0xFF":

        #     raise RuntimeError("Unable to find SGP40; check your wiring")

        self.initialize()

    def initialize(self):
        """Reset the sensor to it's initial unconfigured state and configure it with sensible
        defaults so it can be used"""

        command = bytearray(2)
        command[0] = 0x36
        command[1] = 0x82
        serialnumber = []

        self.readWordFromCommand(command, 2, 10, serialnumber, 3)
        print("Serial Number:")
        for i in serialnumber:
            print("0x{:04X}".format(i), end=" ")

        # uint16_t featureset;
        # command[0] = 0x20;
        # command[1] = 0x2F;
        # if (!readWordFromCommand(command, 2, 10, &featureset, 1))
        #     return false;
        # // print("Featureset 0x"); println(featureset, HEX), end="")

        # VocAlgorithm_init(&voc_algorithm_params);

        # return selfTest();
        self._reset()

    def _reset(self):
        command = bytearray(2)
        command[0] = 0x00
        command[1] = 0x06
        self.readWordFromCommand(command, 2, 10)

    @property
    def raw(self):
        """The raw gas value"""
        # return readWordFromCommand(poo)
        # uint8_t command[8];
        # uint16_t reply;

        # command[0] = 0x26;
        # command[1] = 0x0F;

        # uint16_t rhticks = (uint16_t)((humidity * 65535) / 100 + 0.5);
        # command[2] = rhticks >> 8;
        # command[3] = rhticks & 0xFF;
        # command[4] = generateCRC(command + 2, 2);
        # uint16_t tempticks = (uint16_t)(((temperature + 45) * 65535) / 175);
        # command[5] = tempticks >> 8;
        # command[6] = tempticks & 0xFF;
        # command[7] = generateCRC(command + 5, 2);
        # ;

        # if (!readWordFromCommand(command, 8, 250, &reply, 1))
        #     return 0x0;

        # return reply;
        # }




        return 101

    def readWordFromCommand(self, command_buffer, commandLength, delay_ms, readdata_buffer=None,
                           readlen=None):
        """readWordFromCommand - send a given command code and read the result back

        Args:
            command_buffer (bytearray): The bytarray with the given command bytes
            commandLength (int): The length of the command buffer in bytes
            delay_ms (int): The delay between write and read, in milliseconds
            readdata_buffer (bytearray, optional): The buffer to read the results of the command into. Defaults to None.
            readlen (int, optional): The number of bytes to read. Defaults to None.
        """

        print("Command Buffer:")
        print(["0x{:02X}".format(i) for i in command_buffer])
        with self.i2c_device as i2c:
            i2c.write(command_buffer)

        sleep(round(delay_ms*0.001, 3))

        if (readlen == None):
            return None

        # The number of bytes to rad back, based on the number of words to read
        replylen = readlen * (_WORD_LEN + 1)
        # recycle buffer for read/write w/length
        replybuffer = bytearray(replylen)

        with self.i2c_device as i2c:
            i2c.readinto(replybuffer, end=replylen)

        print("Buffer:")
        print(["0x{:02X}".format(i) for i in replybuffer])

        for i in range(0, replylen, 3):
            if not self._check_crc8(replybuffer[i : i + 2], replybuffer[i + 2]):
                raise RuntimeError("CRC check failed while reading data")
            readdata_buffer.append(unpack_from(">H", replybuffer[i : i + 2])[0])


    @staticmethod
    def _check_crc8(crc_buffer, crc_value):
        crc = 0xFF
        for byte in crc_buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        return crc_value == (crc & 0xFF)  # check against the bottom 8 bits
