# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_sgp40`
================================================================================

CircuitPython library for the Adafruit SGP40 Air Quality Sensor / VOC Index Sensor Breakouts


* Author(s): Bryan Siepert
             Keith Murray

Implementation Notes
--------------------

**Hardware:**

* Adafruit SGP40 Air Quality Sensor Breakout - VOC Index <https://www.adafruit.com/product/4829>

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases


 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

"""
from time import sleep
from struct import unpack_from
import adafruit_bus_device.i2c_device as i2c_device

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_SGP40.git"

_WORD_LEN = 2
# no point in generating this each time
_READ_CMD = [
    0x26,
    0x0F,
    0x80,
    0x00,
    0xA2,
    0x66,
    0x66,
    0x93,
]  # Generated from temp 25c, humidity 50%


class SGP40:
    """
    Class to use the SGP40 Ambient Light and UV sensor

    :param ~busio.I2C i2c: The I2C bus the SGP40 is connected to.
    :param int address: The I2C address of the device. Defaults to :const:`0x59`


    **Quickstart: Importing and using the SGP40 temperature sensor**

        Here is one way of importing the `SGP40` class so you can use it with the name ``sgp``.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import busio
            import board
            import adafruit_sgp40

        Once this is done you can define your `busio.I2C` object and define your sensor object

        .. code-block:: python

            i2c = busio.I2C(board.SCL, board.SDA)
            sgp = adafruit_sgp40.SGP40(i2c)

        Now you have access to the raw gas value using the :attr:`raw` attribute

        .. code-block:: python

            raw_gas_value = sgp.raw


    """

    def __init__(self, i2c, address=0x59):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        self._command_buffer = bytearray(2)
        self._measure_command = _READ_CMD

        self.initialize()

    def initialize(self):
        """Reset the sensor to it's initial unconfigured state and configure it with sensible
        defaults so it can be used"""
        # check serial number
        self._command_buffer[0] = 0x36
        self._command_buffer[1] = 0x82
        serialnumber = self._read_word_from_command(3)

        if serialnumber[0] != 0x0000:
            raise RuntimeError("Serial number does not match")

        # Check feature set
        self._command_buffer[0] = 0x20
        self._command_buffer[1] = 0x2F
        featureset = self._read_word_from_command()
        if featureset[0] != 0x3220:

            raise RuntimeError("Feature set does not match: %s" % hex(featureset[0]))

        # VocAlgorithm_init(&voc_algorithm_params)

        # Self Test
        self._command_buffer[0] = 0x28
        self._command_buffer[1] = 0x0E
        self_test = self._read_word_from_command(delay_ms=250)
        if self_test[0] != 0xD400:
            raise RuntimeError("Self test failed")
        self._reset()

    def _reset(self):
        # This is a general call Reset. Several sensors may see this and it doesn't appear to
        # ACK before resetting
        self._command_buffer[0] = 0x00
        self._command_buffer[1] = 0x06
        try:
            self._read_word_from_command(delay_ms=50)
        except OSError:
            # print("\tGot expected OSError from reset")
            pass
        sleep(1)

    @property
    def raw(self):
        """The raw gas value"""
        # recycle a single buffer
        self._command_buffer = self._measure_command
        read_value = self._read_word_from_command(delay_ms=250)
        self._command_buffer = bytearray(2)
        return read_value[0]

    def _read_word_from_command(
        self,
        delay_ms=10,
        readlen=1,
    ):
        """_read_word_from_command - send a given command code and read the result back

        Args:
            delay_ms (int, optional): The delay between write and read, in milliseconds.
                Defaults to 10ms
            readlen (int, optional): The number of bytes to read. Defaults to 1.
        """
        # TODO: Take 2-byte command as int (0x280E, 0x0006) and packinto command buffer

        with self.i2c_device as i2c:
            i2c.write(self._command_buffer)

        sleep(round(delay_ms * 0.001, 3))

        if readlen is None:
            return None
        readdata_buffer = []

        # The number of bytes to read back, based on the number of words to read
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

    def _check_crc8(self, crc_buffer, crc_value):
        return crc_value == self._generate_crc(crc_buffer)

    @staticmethod
    def _temp_c_to_ticks(temperature):
        """
        tests : From SGP40 Datasheet Table 10
        temp (C)    | Hex Code (Check Sum/CRC Hex Code)
            25      | 0x6666   (CRC 0x93)
            -45     | 0x0000   (CRC 0x81)
            130     | 0xFFFF   (CRC 0xAC)

        """
        temp_ticks = int(((temperature + 45) * 65535) / 175) & 0xFFFF
        least_sig_temp_ticks = temp_ticks & 0xFF
        most_sig_temp_ticks = (temp_ticks >> 8) & 0xFF

        return [most_sig_temp_ticks, least_sig_temp_ticks]

    @staticmethod
    def _relative_humidity_to_ticks(humidity):
        """
        tests : From SGP40 Datasheet Table 10
        Humidity (%) | Hex Code (Check Sum/CRC Hex Code)
            50       | 0x8000   (CRC 0xA2)
            0        | 0x0000   (CRC 0x81)
            100      | 0xFFFF   (CRC 0xAC)

        """
        humidity_ticks = int((humidity * 65535) / 100 + 0.5) & 0xFFFF
        least_sig_rhumidity_ticks = humidity_ticks & 0xFF
        most_sig_rhumidity_ticks = (humidity_ticks >> 8) & 0xFF

        return [most_sig_rhumidity_ticks, least_sig_rhumidity_ticks]

    def measure_raw(self, temperature=25, relative_humidity=50):
        """
        The raw gas value adjusted for the current temperature (c) and humidity (%)
        """
        # recycle a single buffer
        _compensated_read_cmd = [0x26, 0x0F]
        humidity_ticks = self._relative_humidity_to_ticks(relative_humidity)
        humidity_ticks.append(self._generate_crc(humidity_ticks))
        temp_ticks = self._temp_c_to_ticks(temperature)
        temp_ticks.append(self._generate_crc(temp_ticks))
        _cmd = _compensated_read_cmd + humidity_ticks + temp_ticks
        self._measure_command = bytearray(_cmd)
        # print(self._command_buffer)
        # read_value = self._read_word_from_command(delay_ms=250)
        # self._command_buffer = bytearray(2)
        return self.raw

    @staticmethod
    def _generate_crc(crc_buffer):
        crc = 0xFF
        for byte in crc_buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (
                        crc << 1
                    ) ^ 0x31  #  0x31 is the Seed for SGP40's CRC polynomial
                else:
                    crc = crc << 1
        return crc & 0xFF  # Returns only bottom 8 bits
