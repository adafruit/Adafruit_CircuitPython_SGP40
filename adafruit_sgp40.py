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
    _id_reg = ROUnaryStruct(0xFF, "<B")
    # _uvs_data_reg = UnalignedStruct(_UVSDATA, "<I", 24, 3)  # bits, bytes to read/write
    """Ask the sensor if new data is available"""

    def __init__(self, i2c, address=0x59):

        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        if self._id_reg != "0xFF":

            raise RuntimeError("Unable to find SGP40; check your wiring")

        self.initialize()

    def initialize(self):
        """Reset the sensor to it's initial unconfigured state and configure it with sensible
        defaults so it can be used"""

        self._reset()
        self._enable_bit = True
        if not self._enable_bit:
            raise RuntimeError("Unable to enable sensor")
        self._mode = UV
        self.gain = Gain.GAIN_3X

    def _reset(self):
        try:
            self._reset_bit = True
        except OSError:
            # The write to the reset bit will fail because it seems to not ACK before it resets
            pass

        sleep(0.1)
        # check that reset is complete w/ the bit unset
        if self._reset_bit:
            raise RuntimeError("Unable to reset sensor")

    @property
    def gain(self):
        """The amount of gain the raw measurements are multiplied by"""
        return self._gain_bits

    @gain.setter
    def gain(self, value):
        if not Gain.is_valid(value):
            raise AttributeError("gain must be a Gain")
        self._gain_bits = value
