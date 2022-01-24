Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-sgp40/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/sgp40/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_SGP40/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_SGP40/actions
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

CircuitPython library for the Adafruit SGP40 Air Quality Sensor / VOC Index Sensor Breakouts


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

.. code-block:: python3

    import time
    import board
    import adafruit_sgp40

    i2c = board.I2C()  # uses board.SCL and board.SDA
    sgp = adafruit_sgp40(i2c)

    while True:
        print("Measurement: ", sgp.raw)
        print("")
        sleep(1)

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

        # For compensated raw gas readings
        """
        compensated_raw_gas = sgp.measure_raw(
            temperature=temperature, relative_humidity=humidity
        )
        print("Raw Data:", compensated_raw_gas)
        """

        # For Compensated voc index readings
        voc_index = sgp.measure_index(
        temperature=temperature, relative_humidity=humidity)

        print("VOC Index:", voc_index)
        print("")
        time.sleep(1)

It may take several minutes for the VOC index to start changing as it calibrates the baseline readings.
The voc algorithm expects a 1 hertz sampling rate. Run `measure_index()` once per second.


Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/sgp40/en/latest/>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_SGP40/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
