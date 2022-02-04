# Terpulence II Flight Computer - GPS Test
# Author: Garrett Alessandrini

import time
import datetime
import board
import busio
import digitalio
import serial
import adafruit_gps

# Settings we can mess with - edit these
dev_output = True
delay_between_data = 0.05
current_sea_level_pressure = 1013.25

# Setting up sensors - do not edit
i2c = board.I2C()
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
gps = adafruit_gps.GPS(uart, debug=False)


# Variables for the sensors - do not edit
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,500')


# GPS cache variables
cached_latitude = -1
cached_longitude = -1
cached_altitude = -1
cached_speed = -1


# GPS (no accuracy data)
def check_gps():
    global cached_latitude
    global cached_longitude
    global cached_altitude
    global cached_speed
    if gps.update():
        cached_latitude = gps.latitude
        cached_longitude = gps.longitude
        if gps.altitude_m is not None:
            cached_altitude = gps.altitude_m
        if gps.speed_knots is not None:
            cached_speed = gps.speed_knots
    return cached_latitude, cached_longitude, cached_altitude, cached_speed


# Reads data from all sensors, stores into a log, and transmits data over radio
def read_data():
    # First, get data
    latitude, longitude, gps_altitude, gps_speed = check_gps()

    # Dev Mode: Print data to console
    if dev_output:
        print("GPS:")
        print("Latitude: {0:.6f} degrees".format(latitude))
        print("Longitude: {0:.6f} degrees".format(longitude))
        print("Altitude: {} meters".format(gps_altitude))
        print("Speed: {} knots".format(gps_speed))
        print("")


while True:
    read_data()
    time.sleep(delay_between_data)
