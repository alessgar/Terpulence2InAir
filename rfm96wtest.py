# Terpulence II Flight Computer - Transciever Test
# Author: Garrett Alessandrini

import time
import datetime
import board
import busio
import digitalio
import serial
import adafruit_rfm9x

# Settings we can mess with - edit these
dev_output = True
delay_between_data = 0.05
current_sea_level_pressure = 1013.25

# Setting up sensors - do not edit
i2c = board.I2C()
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi,
                             digitalio.DigitalInOut(board.CE1),
                             digitalio.DigitalInOut(board.D25),
                             433.0,
                             baudrate=1000000)  # last parameter slows down speed for breadboards


# Variables for the sensors - do not edit
rfm9x.tx_power = 23
rfm9x.node = 1
rfm9x.destination = 2
rfm9x.enable_crc = True


# Reads data from all sensors, stores into a log, and transmits data over radio
def read_data():
    # Lastly, transmit information over radio
    rfm9x.send(bytes(
        "Hello From Script 1!", "UTF-8"))


while True:
    read_data()
    time.sleep(delay_between_data)
