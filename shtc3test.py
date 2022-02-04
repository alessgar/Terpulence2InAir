# Terpulence II Flight Computer - SHTC3 Test
# Author: Garrett Alessandrini

import time
import datetime
import board
import busio
import digitalio
import serial
import adafruit_shtc3

# Settings we can mess with - edit these
dev_output = True
delay_between_data = 0.05
current_sea_level_pressure = 1013.25

# Setting up sensors - do not edit
i2c = board.I2C()
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
shtc = adafruit_shtc3.SHTC3(i2c)


# Thermometer (+- 0.2C accuracy) + Hygrometer (+=2% accuracy)
def check_shtc():
    temperature, relative_humidity = shtc.measurements
    return temperature, ((temperature * 1.8) + 32), relative_humidity


# Reads data from all sensors, stores into a log, and transmits data over radio
def read_data():
    # First, get data
    temperature_c, temperature_f, relative_humidity = check_shtc()

    # Dev Mode: Print data to console
    if dev_output:
        # SHTC3
        print("SHTC3:")
        print("Temperature (C): %0.1f C" % temperature_c)
        print("Temperature (F): %0.1f F" % temperature_f)
        print("Humidity: %0.1f %%" % relative_humidity)
        print("")


while True:
    read_data()
    time.sleep(delay_between_data)
