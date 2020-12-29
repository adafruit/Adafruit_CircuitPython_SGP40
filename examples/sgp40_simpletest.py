# SPDX-FileCopyrightText: 2020 by Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import adafruit_sgp40

i2c = busio.I2C(board.SCL, board.SDA)
sgp = adafruit_sgp40.SGP40(i2c)
print("Found SGP40 serial #")
# print(hex(sgp.serialnumber[0]), end=" ")
# print(hex(sgp.serialnumber[1]), end=" ")
# print(hex(sgp.serialnumber[2]))
# print("")

while True:
    print("Measurement: ", sgp.raw)
    print("")
    time.sleep(1)
