Introduction
============

CircuitPython library for the Adafruit SGP40 Air Quality Sensor / VOC Index Sensor Breakouts. 
This fork adds DFRobots VOC Index algorithm to adafruits CircuitPython library.

`Original Library <https://github.com/adafruit/Adafruit_CircuitPython_SGP40>`_
`DFRobot Algorithm <https://github.com/DFRobot/DFRobot_SGP40>`_


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_.

Installing from PyPI
=====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-sgp40/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-sgp40

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-sgp40

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-sgp40

Usage Example
=============

For humidity compensated raw gas and voc index readings, we'll need a secondary sensor such as the bme280

.. code-block:: python3

    import time
    import board
    import adafruit_sgp40
    import adafruit_bme280

    i2c = board.I2C()  # uses board.SCL and board.SDA
    sgp = adafruit_sgp40.SGP40(i2c)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

    while True:
        temperature = bme280.temperature
        humidity = bme280.relative_humidity

        # Compensated raw gas reading
        compensated_raw_gas = sgp.measure_index(temperature = temperature, relative_humidity = humidity)

        # Compensated voc index reading
        voc_index = sgp.measure_raw(temperature = temperature, relative_humidity = humidity)

        print(compensated_raw_gas)
        print(voc_index)
        print("")
        time.sleep(1)


It may take several minutes for the VOC index to start changing as it calibrates the baseline readings.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_SGP40/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
