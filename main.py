# Terpulence II Flight Computer - In-Air
# Author: Garrett Alessandrini

import time
import datetime
import board
import busio
import digitalio
import serial
import adafruit_shtc3
import adafruit_bmp3xx
import adafruit_bno055
import adafruit_gps
import adafruit_rfm9x

# Settings we can mess with - edit these
dev_output = True
delay_between_data = 0.05
current_sea_level_pressure = 1013.25

# Setting up sensors - do not edit
i2c = board.I2C()
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
shtc = adafruit_shtc3.SHTC3(i2c)
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
bno = adafruit_bno055.BNO055_I2C(i2c)
gps = adafruit_gps.GPS(uart, debug=False)
rfm9x = adafruit_rfm9x.RFM9x(spi,
                             digitalio.DigitalInOut(board.CE1),
                             digitalio.DigitalInOut(board.D25),
                             433.0,
                             baudrate=1000000)  # last parameter slows down speed for breadboards


# Variables for the sensors - do not edit
bmp.pressure_oversampling = 8
bmp.temperature_oversampling = 2
bmp.sea_level_pressure = current_sea_level_pressure
startup_timestamp = time.time()
startup_altitude = -1
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,500')
rfm9x.tx_power = 23
rfm9x.node = 1
rfm9x.destination = 2
rfm9x.enable_crc = True


# Thermometer (+- 0.2C accuracy) + Hygrometer (+=2% accuracy)
def check_shtc():
    temperature, relative_humidity = shtc.measurements
    return temperature, ((temperature * 1.8) + 32), relative_humidity


# Barometer (+-8 pascals accuracy) + Altimeter (+- 0.5 meter accuracy)
def check_bmp():
    return bmp.pressure, bmp.altitude, (bmp.altitude * 3.2808)


# IMU (no accuracy data, but it apparently self-calibrates)
def check_bno():
    return bno.acceleration, bno.linear_acceleration, bno.gravity, bno.gyro, \
        bno.euler, bno.quaternion, bno.magnetic


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
    temperature_c, temperature_f, relative_humidity = check_shtc()
    pressure, altitude_m, altitude_f = check_bmp()
    acceleration, linear_acceleration, gravity, angular_velocity, orientation_e, \
        orientation_q, magnetic_field_strength = check_bno()
    latitude, longitude, gps_altitude, gps_speed = check_gps()

    # Dev Mode: Print data to console
    if dev_output:
        # SHTC3
        print("SHTC3:")
        print("Temperature (C): %0.1f C" % temperature_c)
        print("Temperature (F): %0.1f F" % temperature_f)
        print("Humidity: %0.1f %%" % relative_humidity)
        print("BMP388:")
        print("Pressure: %6.1f" % pressure)
        print("Altitude (meters): %5.2f meters" % altitude_m)
        print("Altitude (feet): %5.2f feet" % altitude_f)
        print("BNO055:")
        print("Accelerometer: {} m/s^2".format(acceleration))
        print("Linear Acceleration: {} m/s^2".format(linear_acceleration))
        print("Gravity: {} m/s^2".format(gravity))
        print("Angular Velocity: {} rad/sec".format(angular_velocity))
        print("Euler Orientation: {}".format(orientation_e))
        print("Quaternion Orientation: {}".format(orientation_q))
        print("Magnetic Field Strength: {} microteslas".format(magnetic_field_strength))
        print("GPS:")
        print("Latitude: {0:.6f} degrees".format(latitude))
        print("Longitude: {0:.6f} degrees".format(longitude))
        print("Altitude: {} meters".format(gps_altitude))
        print("Speed: {} knots".format(gps_speed))
        print("")

    # Write data to disk
    log = open("log_" + str(startup_timestamp) + ".txt", "a")

    # Start header w/ timestamp
    log.write("START_READING " + datetime.datetime.now().strftime('%H:%M:%S:%f'))

    # SHTC3
    log.write("%0.1f" % temperature_c)
    log.write("Temperature (F): %0.1f" % temperature_f)
    log.write("%0.1f" % relative_humidity)
    log.write("%6.1f" % pressure)
    log.write("%5.2f" % altitude_m)
    log.write("%5.2f" % altitude_f)
    log.write(str(acceleration))
    log.write(str(linear_acceleration))
    log.write(str(gravity))
    log.write(str(angular_velocity))
    log.write(str(orientation_e))
    log.write(str(orientation_q))
    log.write(str(magnetic_field_strength))
    log.write("{0:.6f}".format(latitude))
    log.write("{0:.6f}".format(longitude))
    log.write(str(gps_altitude))
    log.write(str(gps_speed))

    # End header
    log.write("END_READING")
    log.close()

    # Lastly, transmit information over radio
    rfm9x.send(bytes(
        str(temperature_c) + ";"
        + str(temperature_f) + ";"
        + str(relative_humidity) + ";"
        + str(pressure) + ";"
        + str(altitude_m) + ";"
        + str(altitude_f) + ";"
        + str(acceleration) + ";"
        + str(linear_acceleration) + ";"
        + str(gravity) + ";"
        + str(angular_velocity) + ";"
        + str(orientation_e) + ";"
        + str(orientation_q) + ";"
        + str(magnetic_field_strength) + ";"
        + str(latitude) + ";"
        + str(longitude) + ";"
        + str(gps_altitude) + ";"
        + str(gps_speed), "UTF-8"))


while True:
    read_data()
    time.sleep(delay_between_data)
