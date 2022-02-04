# Terpulence II Flight Computer - BMP388 Test
# Author: Garrett Alessandrini

import time
import datetime
import board
import busio
import digitalio
import serial
import adafruit_bmp3xx

# Settings we can mess with - edit these
dev_output = True
delay_between_data = 0.05
current_sea_level_pressure = 1013.25

# Setting up sensors - do not edit
i2c = board.I2C()
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)


# Variables for the sensors - do not edit
bmp.pressure_oversampling = 8
bmp.temperature_oversampling = 2
bmp.sea_level_pressure = current_sea_level_pressure


# Barometer (+-8 pascals accuracy) + Altimeter (+- 0.5 meter accuracy)
def check_bmp():
    return bmp.pressure, bmp.altitude, (bmp.altitude * 3.2808)


# Reads data from all sensors, stores into a log, and transmits data over radio
def read_data():
    # First, get data
    pressure, altitude_m, altitude_f = check_bmp()

    # Dev Mode: Print data to console
    if dev_output:
        print("BMP388:")
        print("Pressure: %6.1f" % pressure)
        print("Altitude (meters): %5.2f meters" % altitude_m)
        print("Altitude (feet): %5.2f feet" % altitude_f)
        print("")


while True:
    read_data()
    time.sleep(delay_between_data)
