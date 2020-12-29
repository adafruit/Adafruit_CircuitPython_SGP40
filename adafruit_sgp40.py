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


class SGP40:
    """Class to use the SGP40 Ambient Light and UV sensor"""

    # _reset_bit = RWBit(_CTRL, 4)
    # _gain_bits = RWBits(3, _GAIN, 0)
    # _id_reg = ROUnaryStruct(0xFF, "<B")
    # _uvs_data_reg = UnalignedStruct(_UVSDATA, "<I", 24, 3)  # bits, bytes to read/write
    """Ask the sensor if new data is available"""

    def __init__(self, i2c, address=0x59):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        self._command_buffer = bytearray(2)

        self.initialize()

    def initialize(self):
        """Reset the sensor to it's initial unconfigured state and configure it with sensible
        defaults so it can be used"""

        self._command_buffer[0] = 0x36
        self._command_buffer[1] = 0x82
        serialnumber = self.readWordFromCommand(3)
        print("Serial Number:")
        for i in serialnumber:
            print("0x{:04X}".format(i), end=" ")
        print("")
        self._command_buffer[0] = 0x20
        self._command_buffer[1] = 0x2F
        featureset = self.readWordFromCommand()
        print("Feature Set:")
        for i in featureset:
            print("0x{:04X}".format(i), end=" ")
        print("")

        # VocAlgorithm_init(&voc_algorithm_params)

        # Self Test
        print("Self test:")

        self._command_buffer[0] = 0x28
        self._command_buffer[1] = 0x0E
        reply = self.readWordFromCommand(delay_ms=250)
        if 0xD400 != reply[0]:
            raise RuntimeError("Self test failed")
        print("\tOK\n")
        self._reset()

    def _reset(self):
        # This is a general call Reset. Several sensors may see this and it doesn't appear to
        # ACK before resetting
        self._command_buffer[0] = 0x00
        self._command_buffer[1] = 0x06
        try:
            print("Reset:")
            self.readWordFromCommand()
        except OSError:
            print("\tGot expected OSError from reset")

    @property
    def raw(self):
        """The raw gas value"""
        # uint8_t command[8]
        # uint16_t reply

        # command[0] = 0x26
        # command[1] = 0x0F

        # uint16_t rhticks = (uint16_t)((humidity * 65535) / 100 + 0.5)
        # command[2] = rhticks >> 8
        # command[3] = rhticks & 0xFF
        # command[4] = generateCRC(command + 2, 2)
        # uint16_t tempticks = (uint16_t)(((temperature + 45) * 65535) / 175)
        # command[5] = tempticks >> 8
        # command[6] = tempticks & 0xFF
        # command[7] = generateCRC(command + 5, 2)
        #

        # if (!readWordFromCommand(command, 8, 250, &reply, 1))
        #     return 0x0

        # return reply
        # }

        return 101

    def readWordFromCommand(
        self,
        delay_ms=10,
        readlen=1,
    ):
        """readWordFromCommand - send a given command code and read the result back

        Args:
            delay_ms (int, optional): The delay between write and read, in milliseconds. Defaults to 10ms
            readlen (int, optional): The number of bytes to read. Defaults to 1.
        """

        # print("Command Buffer:")
        # print(["0x{:02X}".format(i) for i in self._command_buffer])
        with self.i2c_device as i2c:
            i2c.write(self._command_buffer)

        sleep(round(delay_ms * 0.001, 3))

        if readlen == None:
            return None
        readdata_buffer = []

        # The number of bytes to rad back, based on the number of words to read
        replylen = readlen * (_WORD_LEN + 1)
        # recycle buffer for read/write w/length
        replybuffer = bytearray(replylen)

        with self.i2c_device as i2c:
            i2c.readinto(replybuffer, end=replylen)

        # print("Buffer:")
        # print(["0x{:02X}".format(i) for i in replybuffer])

        for i in range(0, replylen, 3):
            if not self._check_crc8(replybuffer[i : i + 2], replybuffer[i + 2]):
                raise RuntimeError("CRC check failed while reading data")
            readdata_buffer.append(unpack_from(">H", replybuffer[i : i + 2])[0])

        return readdata_buffer

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
