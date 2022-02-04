# Terpulence II Flight Computer - BNO055 Test
# Author: Garrett Alessandrini

import time
import datetime
import board
import busio
import digitalio
import serial
import adafruit_bno055

# Settings we can mess with - edit these
dev_output = True
delay_between_data = 0.05
current_sea_level_pressure = 1013.25

# Setting up sensors - do not edit
i2c = board.I2C()
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
bno = adafruit_bno055.BNO055_I2C(i2c)


# IMU (no accuracy data, but it apparently self-calibrates)
def check_bno():
    return bno.acceleration, bno.linear_acceleration, bno.gravity, bno.gyro, \
        bno.euler, bno.quaternion, bno.magnetic


# Reads data from all sensors, stores into a log, and transmits data over radio
def read_data():
    # First, get data
    acceleration, linear_acceleration, gravity, angular_velocity, orientation_e, \
        orientation_q, magnetic_field_strength = check_bno()

    # Dev Mode: Print data to console
    if dev_output:
        print("BNO055:")
        print("Accelerometer: {} m/s^2".format(acceleration))
        print("Linear Acceleration: {} m/s^2".format(linear_acceleration))
        print("Gravity: {} m/s^2".format(gravity))
        print("Angular Velocity: {} rad/sec".format(angular_velocity))
        print("Euler Orientation: {}".format(orientation_e))
        print("Quaternion Orientation: {}".format(orientation_q))
        print("Magnetic Field Strength: {} microteslas".format(magnetic_field_strength))
        print("")


while True:
    read_data()
    time.sleep(delay_between_data)
