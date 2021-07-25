import time
import board
import adafruit_sgp40
import adafruit_bme280

# Boards i2c bus
i2c = board.I2C()  # uses board.SCL and board.SDA
sgp = adafruit_sgp40.SGP40(i2c)

# Humidity sensor for compensated Readings
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

while True:
    temperature = bme280.temperature
    humidity = bme280.relative_humidity

    # Compensated voc index reading
    voc_index = sgp.measure_raw(
        temperature=temperature, relative_humidity=humidity)

    print(voc_index)
    print("")
    time.sleep(1)
