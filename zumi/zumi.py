# Zumi python library version 1.66

import os
import smbus2
import time
import configparser
from zumi.protocol import Accelerometer, Gyro, Device, MPURegister, Axis
from zumi.protocol import CommandType, MotorCommand, Command
import RPi.GPIO as GPIO
import math
import logging
import subprocess
from uuid import getnode as get_mac
import numpy as np


def preboot_to_postboot():
    from zumi.util.screen import Screen
    bus = smbus2.SMBus(1)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(4, GPIO.IN)

    while GPIO.input(4) == 1:
        time.sleep(0.01)
        continue

    time.sleep(1)
    arduino_address = 0x04
    try:
        bus.write_byte(arduino_address, 0b11000000)
    except IOError:
        time.sleep(0.01)
        bus.write_byte(arduino_address, 0b11000000)

    screen = Screen()
    screen.draw_image_by_name('wakingup_witheyes')


def _is_in_range(value, low, high):
    return low <= value <= high


def _convert_to_arduino_speed(arduino_speed):
    if arduino_speed < 0:
        arduino_speed *= -1
        arduino_speed += 127
    return arduino_speed


def _create_command(command_type, command, extra_bits=0):
    command_type = command_type << 6
    command = command << 3
    return command_type | command | extra_bits


def _bit_read(byte, index):
    return byte >> index & 1 == 1


def _bit_set(byte, index):
    return byte | (1 << index)


def _bit_clear(byte, index):
    return byte & ~(1 << index)


class ValueOutOfRangeError(Exception):
    pass


# credit to https://github.com/RigacciOrg/py-qmc5883l
class QMC5883L:
    """Interface for the QMC5883l 3-Axis Magnetic Sensor."""

    def __init__(self, bus):

        self.DFLT_ADDRESS = 0x0d  # ADDRESS for the QMC Compass module
        self.REG_XOUT_LSB = 0x00  # Output Data Registers for magnetic sensor.
        self.REG_XOUT_MSB = 0x01
        self.REG_YOUT_LSB = 0x02
        self.REG_YOUT_MSB = 0x03
        self.REG_ZOUT_LSB = 0x04
        self.REG_ZOUT_MSB = 0x05
        self.REG_STATUS_1 = 0x06  # Status Register.
        self.REG_TOUT_LSB = 0x07  # Output Data Registers for temperature.
        self.REG_TOUT_MSB = 0x08
        self.REG_CONTROL_1 = 0x09  # Control Register #1.
        self.REG_CONTROL_2 = 0x0a  # Control Register #2.
        self.REG_RST_PERIOD = 0x0b  # SET/RESET Period Register.
        self.REG_CHIP_ID = 0x0d  # Chip ID register.

        # Flags for Status Register #1.
        self.STAT_DRDY = 0b00000001  # Data Ready.
        self.STAT_OVL = 0b00000010  # Overflow flag.
        self.STAT_DOR = 0b00000100  # Data skipped for reading.

        # Flags for Status Register #2.
        self.INT_ENB = 0b00000001  # Interrupt Pin Enabling.
        self.POL_PNT = 0b01000000  # Pointer Roll-over.
        self.SOFT_RST = 0b10000000  # Soft Reset.

        # Flags for Control Register 1.
        self.MODE_STBY = 0b00000000  # Standby mode.
        self.MODE_CONT = 0b00000001  # Continuous read mode.
        self.ODR_10HZ = 0b00000000  # Output Data Rate Hz.
        self.ODR_50HZ = 0b00000100
        self.ODR_100HZ = 0b00001000
        self.ODR_200HZ = 0b00001100
        self.RNG_2G = 0b00000000  # Range 2 Gauss: for magnetic-clean environments.
        self.RNG_8G = 0b00010000  # Range 8 Gauss: for strong magnetic fields.
        self.OSR_512 = 0b00000000  # Over Sample Rate 512: less noise, more power.
        self.OSR_256 = 0b01000000
        self.OSR_128 = 0b10000000
        self.OSR_64 = 0b11000000  # Over Sample Rate 64: more noise, less power.

        self.output_data_rate = self.ODR_200HZ
        self.output_range = self.RNG_8G
        self.oversampling_rate = self.OSR_512
        self.address = self.DFLT_ADDRESS
        self.bus = bus

        self._declination = 0.0
        self._calibration = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]]

        self.mode_cont = (self.MODE_CONT | self.output_data_rate | self.output_range
                          | self.oversampling_rate)
        self.mode_stby = (self.MODE_STBY | self.ODR_10HZ | self.RNG_2G | self.OSR_64)
        self.mode_continuous()

    def mode_continuous(self):
        """Set the device in continuous read mode."""
        self._write_byte(self.REG_CONTROL_2, self.SOFT_RST)  # Soft reset.
        self._write_byte(self.REG_CONTROL_2, self.INT_ENB)  # Disable interrupt.
        self._write_byte(self.REG_RST_PERIOD, 0x01)  # Define SET/RESET period.
        self._write_byte(self.REG_CONTROL_1, self.mode_cont)  # Set operation mode.

    def mode_standby(self):
        """Set the device in standby mode."""
        self._write_byte(self.REG_CONTROL_2, self.SOFT_RST)
        self._write_byte(self.REG_CONTROL_2, self.INT_ENB)
        self._write_byte(self.REG_RST_PERIOD, 0x01)
        self._write_byte(self.REG_CONTROL_1, self.mode_stby)  # Set operation mode.

    def _write_byte(self, registry, value):
        self.bus.write_byte_data(self.address, registry, value)
        time.sleep(0.005)
        # time.sleep(0.01)

    def _read_byte(self, registry):
        return self.bus.read_byte_data(self.address, registry)

    def _read_word(self, registry):
        """Read a two bytes value stored as LSB and MSB."""
        low = self.bus.read_byte_data(self.address, registry)
        high = self.bus.read_byte_data(self.address, registry + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, registry):
        """Calculate the 2's complement of a two bytes value."""
        val = self._read_word(registry)
        if val >= 0x8000:  # 32768
            return val - 0x10000  # 65536
        else:
            return val

    def get_data(self):
        """Read data from magnetic and temperature data registers."""
        i = 0
        [x, y, z, t] = [None, None, None, None]

        while i < 20:  # Timeout after about 0.20 seconds.
            status = self._read_byte(self.REG_STATUS_1)
            if status & self.STAT_OVL:
                # Some values have reached an overflow.
                msg = "Magnetic sensor overflow."
                if self.output_range == self.RNG_2G:
                    msg += " Consider switching to RNG_8G output range."
                logging.warning(msg)
            if status & self.STAT_DOR:
                # Previous measure was read partially, sensor in Data Lock.
                x = self._read_word_2c(self.REG_XOUT_LSB)
                y = self._read_word_2c(self.REG_YOUT_LSB)
                z = self._read_word_2c(self.REG_ZOUT_LSB)
                continue
            if status & self.STAT_DRDY:
                # Data is ready to read.
                x = self._read_word_2c(self.REG_XOUT_LSB)
                y = self._read_word_2c(self.REG_YOUT_LSB)
                z = self._read_word_2c(self.REG_ZOUT_LSB)
                t = self._read_word_2c(self.REG_TOUT_LSB)
                break
            else:
                time.sleep(0.005)
                # Waiting for DRDY.
                i += 1
        return [x, y, z, t]

    def get_magnet_raw(self):
        """Get the 3 axis values from magnetic sensor."""
        [x, y, z, t] = self.get_data()
        return [x, y, z]

    def get_magnet(self):
        """Return the horizontal magnetic sensor vector with (x, y) calibration applied."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            [x1, y1] = [x, y]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + c[1][2]
        return [x1, y1]

    def get_bearing_raw(self):
        """Horizontal bearing (in degrees) from magnetic value X and Y."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            return b

    def get_bearing(self):
        """Horizontal bearing, adjusted by calibration and declination."""
        [x, y] = self.get_magnet()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            b += self._declination
            if b < 0.0:
                b += 360.0
            elif b >= 360.0:
                b -= 360.0
        return b

    def get_temp(self):
        """Raw (uncalibrated) data from temperature sensor."""
        [x, y, z, t] = self.get_data()
        return t

    def set_declination(self, value):
        """Set the magnetic declination, in degrees."""
        # magnetic declination http://www.magnetic-declination.com/what-is-magnetic-declination.php
        # use this site to find declination http://www.magnetic-declination.com/
        try:
            d = float(value)
            if d < -180.0 or d > 180.0:
                logging.error(u'Declination must be >= -180 and <= 180.')
            else:
                self._declination = d
        except:
            logging.error(u'Declination must be a float value.')

    def get_declination(self):
        """Return the current set value of magnetic declination."""
        return self._declination

    def set_calibration(self, value):
        """Set the 3x3 matrix for horizontal (x, y) magnetic vector calibration."""
        c = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        try:
            for i in range(0, 3):
                for j in range(0, 3):
                    c[i][j] = float(value[i][j])
            self._calibration = c
        except:
            logging.error(u'Calibration must be a 3x3 float matrix.')

    def get_calibration(self):
        """Return the current set value of the calibration matrix."""
        return self._calibration

    declination = property(fget=get_declination,
                           fset=set_declination,
                           doc=u'Magnetic declination to adjust bearing.')

    calibration = property(fget=get_calibration,
                           fset=set_calibration,
                           doc=u'Transformation matrix to adjust (x, y) magnetic vector.')


class MPU:
    def __init__(self, bus):
        """
        Initialize registers for the MPU6050. Please consult the MPU6050 spec sheet for more details
        """
        self.bus = bus
        self.offset_path = "/home/pi/offsets.txt"

        """
        Register DIV / 19
        Determines sample rate by dividing the gyroscope output rate by the sent value. By default, the gyroscope 
        outputs at 8khz, the same as the internal oscillator

        Register PWR_M / 107
        Determines power source and clock source. Currently, the rightmost bit is set, meaning that the X axis of the
        gyroscope is used to help synchronize the clock

        Register CONFIG / 26
        Configures the MPU6050 chip. Specifically, it allows one to capture the sample rate of an external device
        using the FSYNC pin, and allows one to set a Digital Low Pass Filter. Currently, all bits are cleared, meaning
        that neither feature is enabled

        Register GYRO_CONFIG / 27
        Configures the gyroscope. Specifically, it allows one to set the full range of the gyroscopes' readings, as
        well as allow the gyroscope to perform a self-diagnostics test. Currently, only bits 3 and 4 are set, meaning
        that the gyroscope can read a max/min of 2000/-2000 degrees per second, and the gyroscope will not perform a
        self-diagnostics test

        Register INT_EN / 56
        Allows one to enable interruptions from different sources. Currently, the rightmost bit is set, which means
        that an interrupt occurs whenever data is ready, or when all the sensor registers have been written to
        """
        self.ACCEL_CONFIG = 0x1C
        self.GYRO_CONFIG = 0x1B

        self.PWR_MGMT_1 = 0x6B
        self.PWR_MGMT_2 = 0x6C

        self.DIV = 0x19
        self.CONFIG = 0x1A
        self.INT_EN = 0x38

        # Pre-defined ranges I2C registers
        self.ACCEL_RANGE_2G = 0x00
        self.ACCEL_RANGE_4G = 0x08
        self.ACCEL_RANGE_8G = 0x10
        self.ACCEL_RANGE_16G = 0x18

        # Scale Modifiers to convert to G
        self.ACCEL_SCALE_MODIFIER_2G = 16384.0
        self.ACCEL_SCALE_MODIFIER_4G = 8192.0
        self.ACCEL_SCALE_MODIFIER_8G = 4096.0
        self.ACCEL_SCALE_MODIFIER_16G = 2048.0
        # -----------------------
        self.GYRO_RANGE_250DEG = 0x00
        self.GYRO_RANGE_500DEG = 0x08
        self.GYRO_RANGE_1000DEG = 0x10
        self.GYRO_RANGE_2000DEG = 0x18

        # scale modifiers to convert to degrees per second
        self.GYRO_SCALE_MODIFIER_250DEG = 131.0
        self.GYRO_SCALE_MODIFIER_500DEG = 65.5
        self.GYRO_SCALE_MODIFIER_1000DEG = 32.8
        self.GYRO_SCALE_MODIFIER_2000DEG = 16.4

        self.DISABLE_FSYNC = 0

        # write to sample rate register
        self.bus.write_byte_data(Device.MPU, self.DIV, 7)

        # wakeup the MPU6050
        self.bus.write_byte_data(Device.MPU, self.PWR_MGMT_1, 0x00)

        # disable FSYNC, set 260 hz acc filtering and 256 hz Gyro filtering
        self.bus.write_byte_data(Device.MPU, self.CONFIG, self.DISABLE_FSYNC)

        # set gyro range to +-1000 degrees a second
        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_1000DEG)

        # set accel range to +-2G
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_2G)

        time.sleep(0.05)

        # just calls Axis object with the correct scalar parameter
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_1000DEG)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_2G)

        # If offset file exists and mac address matches, read calibration
        if self.check_mpu_offsets(self.offset_path):
            print("Gyroscope previously calibrated")

            lines = [line.rstrip('\n') for line in open(self.offset_path)]
            self.calibrate_gyro.X = float(lines[1])
            self.calibrate_gyro.Y = float(lines[2])
            self.calibrate_gyro.Z = float(lines[3])
            self.calibrate_accel.X = float(lines[4])
            self.calibrate_accel.Y = float(lines[5])
            self.calibrate_accel.Z = float(lines[6])

        else:
            print("Calibrating gyroscope now")
            self.calibrate_MPU()

    def set_gyro_250_scale(self):
        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_250DEG)
        time.sleep(0.1)
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_250DEG)
        self.calibrate_MPU()

    def set_gyro_500_scale(self):
        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_500DEG)
        time.sleep(0.1)
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_500DEG)
        self.calibrate_MPU()

    def set_gyro_1000_scale(self):
        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_1000DEG)
        time.sleep(0.1)
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_1000DEG)
        self.calibrate_MPU()

    def set_gyro_2000_scale(self):
        self.bus.write_byte_data(Device.MPU, self.GYRO_CONFIG, self.GYRO_RANGE_2000DEG)
        time.sleep(0.1)
        self.calibrate_gyro = Axis(scalar=self.GYRO_SCALE_MODIFIER_2000DEG)
        self.calibrate_MPU()

    def set_acc_2g_scale(self):
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_2G)
        time.sleep(0.1)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_2G)

    def set_acc_4g_scale(self):
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_4G)
        time.sleep(0.1)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_4G)

    def set_acc_8g_scale(self):
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_8G)
        time.sleep(0.1)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_8G)

    def set_acc_16g_scale(self):
        self.bus.write_byte_data(Device.MPU, self.ACCEL_CONFIG, self.ACCEL_RANGE_16G)
        time.sleep(0.1)
        self.calibrate_accel = Axis(scalar=self.ACCEL_SCALE_MODIFIER_16G)

    def check_mpu_offsets(self, offset_path):
        calibrated = False
        # First check if offset file exists
        if os.path.isfile(offset_path):
            # print("Gyro offset file already exists.")
            lines = [line.rstrip('\n') for line in open(offset_path)]
            # Check if the number makes sense
            if float(lines[0]) > 0:
                # Check if current mac address is same
                if get_mac() - int(float(lines[0])) == 0:
                    print("Verified Pi Zero is the same")
                    calibrated = True
                else:
                    print("Pi Zero is different.")
        return calibrated

    def print_offsets(self):
        print("gyro offsets", self.calibrate_gyro.X, self.calibrate_gyro.Y, self.calibrate_gyro.Z)
        print("accel offsets", self.calibrate_accel.X, self.calibrate_accel.Y, self.calibrate_accel.Z)

    def calibrate_MPU(self, count=100):
        """
        Reads from previous offsets unless file does not exist or recalibrate is manually set to True.
        Take the average of multiple readings from a sensor. The result is the error of the sensor
        Args: Number of sensor readings to take and a boolean for recalibration.
        """
        print("Creating new offset file...")

        for i in range(count):
            if i % 50 == 0:
                print(".")
            current_accel = self.read_multiple_raw_MPU_data(Accelerometer.X, Accelerometer.Y, Accelerometer.Z)
            current_gyro = self.read_multiple_raw_MPU_data(Gyro.X, Gyro.Y, Gyro.Z)
            self.calibrate_accel.add_value(current_accel)
            self.calibrate_gyro.add_value(current_gyro)

        self.calibrate_gyro.calculate_error(count)
        self.calibrate_accel.calculate_error(count)
        self.write_txt_data()

    def write_txt_data(self):
        file = open("/home/pi/offsets.txt", 'w')

        # Write mac address first
        mac = get_mac()
        file.write(str(mac) + "\n")

        # Write mpu6050 data
        file.write(str(self.calibrate_gyro.X) + "\n")
        file.write(str(self.calibrate_gyro.Y) + "\n")
        file.write(str(self.calibrate_gyro.Z) + "\n")
        file.write(str(self.calibrate_accel.X) + "\n")
        file.write(str(self.calibrate_accel.Y) + "\n")
        file.write(str(self.calibrate_accel.Z) + "\n")
        file.close()

    def get_temp(self):
        """
        Reads the temperature from the on-board temperature
        sensor of the MPU-6050.
        Returns the temperature in degrees Celcius.
        :return actual_temp
        """
        TEMP_OUT0 = 0x41
        raw_temp = self.__read_raw_MPU_data(TEMP_OUT0)
        # convert to celsius
        actual_temp = (raw_temp / 340) + 36.53
        return actual_temp

    def get_z_ang_speed(self):
        """
        Reads the raw bit value of the z angular speed
        from the MPU-6050 registers. Will need to be
        scaled to get degrees per second.
        :return: z_angular_speed
        """
        GYRO_ZOUT0 = 0x47
        z_angular_speed = self.__read_raw_MPU_data(GYRO_ZOUT0)
        return z_angular_speed

    def __read_raw_MPU_data(self, addr):
        """
        Read a high and low byte from the MPU6050
        Args: 1 hex address, from which the MPU6050 will obtain a sensor reading
        """
        try:
            high = self.bus.read_byte_data(Device.MPU, addr)
            low = self.bus.read_byte_data(Device.MPU, addr + 1)
            value = ((high << 8) | low)
            if value > 32768:
                value = value - 65536
        except IOError:
            time.sleep(0.01)
            return self.__read_raw_MPU_data()
        return value

    def read_multiple_raw_MPU_data(self, *args):
        """
        Read a high and low byte from the MPU6050
        Args: 1 hex address, from which the MPU6050 will obtain a sensor reading
        """
        result = []
        for item in args:
            result.append(self.__read_raw_MPU_data(item))
        return result

    def read_MPU_data(self, data_index):
        """
        Returns a single MPU6050 reading
        Args: 1 integer, from 0 to 5. Each number corresponds to a different MPU6050 sensor reading:

        0 / Accelerometer X-axis
        1 / Accelerometer Y-axis
        2 / Accelerometer Z-axis
        3 / Gyroscope X-axis
        4 / Gyroscope Y-axis
        5 / Gyroscope Z-axis
        """
        data = self.read_all_MPU_data()
        return data[data_index]

    def read_all_MPU_data(self):
        """
        Returns the calibrated readings for the accelerometer and gyroscope in the form of a list. Use
        get_mpu_data(data_index) to return a single value instead
        Args: None
        """
        data = self.read_multiple_raw_MPU_data(Accelerometer.X, Accelerometer.Y, Accelerometer.Z,
                                               Gyro.X, Gyro.Y, Gyro.Z)
        scalar_data = [self.calibrate_accel.scalar, self.calibrate_accel.scalar, self.calibrate_accel.scalar,
                       self.calibrate_gyro.scalar, self.calibrate_gyro.scalar, self.calibrate_gyro.scalar]
        calibration_data = [0, 0, 0, self.calibrate_gyro.X, self.calibrate_gyro.Y, self.calibrate_gyro.Z]

        # convert raw 10 bit data to into correct units, g for acceleration
        data = [data / scalar_data for data, scalar_data in zip(data, scalar_data)]

        # Convert gyro data into degrees/second
        # remove the offset for the data points.
        data = [data - calibration_data for data, calibration_data in zip(data, calibration_data)]

        return data


class Zumi:
    def __init__(self, skip_preboot=False):
        print("Starting Zumi ")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(4, GPIO.IN)

        if skip_preboot is True:
            self.bus = smbus2.SMBus(1)
        else:
            # verify the zumi board is no longer i2c master
            self.check_arduino_is_master()
            # create i2c bus object
            self.bus = smbus2.SMBus(1)
            # send over i2c msg to disable preboot
            self.disable_arduino_preboot()

        self.MAX_USER_SPEED = 80
        self.MIN_I2C_DELAY = 0.01

        self.start_time = time.time()
        # stop zumi in case it still is driving
        self.stop()
        self.LED_user_pattern = 0
        self.LED_signals = 0
        # reset the user controlled LED's
        self.all_lights_off()
        # turn off any note played
        self.play_note(0, 0)

        self.mpu_list = [0, 0, 0, 0, 0, 0, 0]
        self.angle_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.ir_list = [0, 0, 0, 0, 0, 0]
        self.motor_speeds = [0, 0, time.time() - self.start_time]
        # holds the time since the last mpu reading
        self.gyro_time_past = time.time()
        self.PID_time_past = time.time()

        self.IR_THRESHOLD_LIST = [100,100,100,100,100,100]
        self.IR_front_right = 0
        self.IR_bottom_right = 1
        self.IR_back_right = 2
        self.IR_bottom_left = 3
        self.IR_back_left = 4
        self.IR_front_left = 5

        self.error_sum = 0
        self.error_past = 0

        if os.path.isfile("/home/pi/speed_prediction.txt"):
            lines = [line.rstrip('\n') for line in open("/home/pi/speed_prediction.txt")]
            # predicted speed in inches per second for speed prediction
            self.PRED_SPEED_INCHES_SEC = float(lines[0])
            # predicted slope intercept for speed prediction
            self.PRED_SLOPE_INT_INCH = float(lines[1])
            # the speed which the motors have to be set
            # in order to use the speed prediction
            self.PRED_SET_SPEED = float(lines[2])
        else:
            self.PRED_SPEED_INCHES_SEC = 6
            self.PRED_SLOPE_INT_INCH = 0
            self.PRED_SET_SPEED = 40

        if os.path.isfile("/home/pi/ir_data.txt"):
            ir_calibration_data = np.loadtxt("/home/pi/ir_data.txt")
            self.IR_THRESHOLD_LIST = Zumi.column(2,ir_calibration_data)
        else:
            self.IR_THRESHOLD_LIST = [100, 100, 100, 100, 100, 100]


        # drive PID values; used in forward(),go_straight() etc
        self.D_P = 2.9
        self.D_I = 0.01
        self.D_D = 0.05

        # turn PID values; used in turn()
        self.T_P = 0.6
        self.T_I = 0.001
        self.T_D = 0.001

        # x and y position in centimeters
        self.coordinate = [0, 0]

        self.mpu = MPU(self.bus)
        self.sensors_found = self.check_i2c_sensors()
        self.has_compass = False
        self.board_firmware_version = self.get_board_firmware_version()

        for sensor in self.sensors_found:
            if sensor == 0xd:
                self.has_compass = True

                # if this board has the compass
                # then create the compass object
                self.compass = QMC5883L(self.bus)

        usb_voltage_detected = 1.7
        min_battery_voltage = 3.45
        max_battery_voltage = 4.14
        battery_voltage = self.get_battery_voltage()
        battery_percent = self.get_battery_percent()
        if battery_voltage < usb_voltage_detected:
            print("Zumi is charging")
        elif battery_percent < 25 and battery_voltage > usb_voltage_detected:
            print("Zumi battery level is low ", battery_percent, "%")
            print("Please charge me!")
        elif battery_voltage > min_battery_voltage:
            print("Zumi battery level ", battery_percent, "%")

    @staticmethod
    def stop_zumi():
        print("stopping zumi")
        bus = smbus2.SMBus(1)
        bus.write_byte(0x04, 0b00000000)
        bus.close()

    @staticmethod
    def save_data_txt(sensor_list, file_name="data.txt", append=True):
        message = ""
        print("Saving data")
        if append:
            file = open(file_name, "a")
        else:
            file = open(file_name, "w")
        for val in sensor_list:
            for data_point in val:
                message = message + str(data_point) + " "
            message = message + "\n"
        file.write(message)
        file.close()
        print("Done saving data")

    @staticmethod
    def column(column_number, matrix):
        return [row[column_number] for row in matrix]

    def calibrate_gyro(self):
        """
        easier method to use in order to
        calibrate the gyro
        :return:
        """
        self.mpu.calibrate_MPU()

    def speed_calibration(self, speed=40, ir_threshold=None, time_out=3, cm_per_brick=2, show_graphs=False):
        """
        Will allow Zumi to automatically calculate the best fit line approximation for the true speed in
        inches per second.
        This will only work if the Zumi speed calibration piece is used.
        The speed calibration piece consist of evenly spaced black and white road markers which Zumi's bottom left IR
        sensor detect.

        Will store the values derived to disk and Zumi will remember them.
        Affects the move_inches(), move_centimeter(), and move_coordinate()

        params:
        speed: the set speed at which Zumi will do the best fit line approximation default 40
        ir_threshold: The min value for the bottom left IR sensor to detect the black and white road markers
        time_out: the amount of time Zumi will attempt to do the speed calibration.
        cm_per_brick: the width of each road marker about 2 centimeters
        show_graphs: boolean that allows the graphs to be displayed. defaulted to False

        """
        if ir_threshold is None:
            ir_threshold = self.IR_THRESHOLD_LIST[3]
        # the amount of road_markers you wish to cross over
        road_markers = 5
        samples_taken = 0
        # create list for the readings to be graphed
        ir_list = []
        dbli_list = []
        time_list = []
        y = []
        x = []
        distance_traveled = 0
        # create variables for calculating derivative
        BLI_past = 0
        DBLI = 0
        LBRICKS = 0
        time_passed = 0
        heading = 0
        # reset the gyroscope and the PID accumulators
        self.reset_PID()
        init_time = time.time()
        try:
            while (True):
                self.go_straight(speed, heading)
                time_passed = time.time() - init_time
                ir_readings = self.get_all_IR_data()
                bottom_left_ir = ir_readings[3]
                if bottom_left_ir > ir_threshold:
                    BLI = 1  # this is black
                else:
                    BLI = 0  # this is white

                # change in BLI
                DBLI = BLI - BLI_past

                if DBLI == 1 or DBLI == -1:
                    distance_traveled = distance_traveled + cm_per_brick
                    LBRICKS = LBRICKS + 0.5
                    y.append(distance_traveled)
                    x.append(time_passed)

                # update past Binary Left IR (BLI)
                BLI_past = BLI

                # add data to lists
                dbli_list.append(DBLI)
                time_list.append(time_passed)
                ir_list.append(bottom_left_ir)

                # program should end if timeout or number of bricks exceeded
                if time_passed > time_out or LBRICKS >= road_markers:
                    self.stop()
                    break
        finally:
            self.stop()
            # DONT READ THE FIRST Reading as it skews the results.
        try:
            # find the best fit line in centimeters and seconds
            speed_cm_second, slope_intercept = np.polyfit(x[1:], y[1:], 1)
        except:
            print("Something went wrong")
            print("Please try the speed calibration again \nwith the Speed calibration piece")
            return None

        # convert the current speed calibration values to inches
        self.PRED_SPEED_INCHES_SEC = speed_cm_second / 2.54
        self.PRED_SLOPE_INT_INCH = slope_intercept / 2.54
        self.PRED_SET_SPEED = speed

        print("Save these values")
        print("zumi.PRED_SPEED_INCHES_SEC =", self.PRED_SPEED_INCHES_SEC)
        print("zumi.PRED_SLOPE_INT_INCH =", self.PRED_SLOPE_INT_INCH)
        print("zumi.PRED_SET_SPEED =", self.PRED_SET_SPEED)

        # save the current speed calibration values to disk
        self.save_speed_calibration_values()
        print("Saved values to disk")

        ypred = []
        for i in range(len(x)):
            ypred.append(speed_cm_second * x[i] + slope_intercept)

        if show_graphs:
            import matplotlib.pyplot as plt
            plt.plot(time_list, ir_list)
            plt.ylabel('ir values')
            plt.xlabel('seconds')
            plt.show()

            plt.plot(x, y)
            plt.ylabel('centimeters')
            plt.xlabel('seconds')
            plt.show()

            plt.plot(x, y, 'ro', x, ypred)
            plt.ylabel('predicted centimeter line')
            plt.xlabel('seconds')
            plt.show()

    def save_speed_calibration_values(self):
        """
        Will save the current speed calibration values to disk
        """
        file = open("/home/pi/speed_prediction.txt", 'w')

        # save the speed prediction values to text
        file.write(str(self.PRED_SPEED_INCHES_SEC) + "\n")
        file.write(str(self.PRED_SLOPE_INT_INCH) + "\n")
        file.write(str(self.PRED_SET_SPEED) + "\n")
        file.close()

        p = subprocess.Popen(['sudo', 'chown', 'pi:pi', '/home/pi/speed_prediction.txt'])
        p.communicate()


    def check_i2c_sensors(self):
        """
        Checks for all I2C addresses to determine
        if known addresses have been detected
        a message with the corresponding device
        associated with the address will be output
        :return: sensor_list (list of all i2c devices found)
        """
        sensor_list = []
        # go through all addresses and
        # check if any device found
        for device in range(128):
            try:
                self.bus.read_byte(device)
                sensor_list.append(device)
                if device == 0x3c:
                    print("OLED Screen detected")
                if device == 0x68:
                    print("Gyroscope & Accelerometer detected")
                if device == 0xd:
                    print("Compass detected")
                if device == 0x4:
                    print("Zumi board detected")
            except:  # exception if read_byte fails
                pass
        return sensor_list

    def check_arduino_is_master(self):
        """
        Checks if the Zumi board arduino has finished the pre bootup sequence
        by checking the shared GPIO pin.
        If the pin is still held HIGH the pre boot sequence has not finished
        or the Zumi board is in booted up charge mode.
        if the pin is LOW the pre boot up sequence has finished
        :return: nothing
        """
        if GPIO.input(4) == 1:
            print("Zumi board I2C busy, wait for white LED.")
            print("If Zumi is in charge mode please flip the switch to the on position")
            while GPIO.input(4) == 1:
                time.sleep(0.05)
        print("Pi Zero I2C is available")

    def disable_arduino_preboot(self):
        """
        sends a I2C message to disable the
        Zumi board arduino I2C master mode
        in order for the Pi to be the I2C master
        The pre boot sequence will need to have
        finished in order for this to work
        :return: nothing
        """

        while True:
            try:
                self.bus.write_byte(0x04, 0b11000000)
                break
            except IOError:
                time.sleep(0.05)
                print(" failed to disable arduino preboot ")
                continue

    def reset_PID(self):
        """
        The error values for error integral and error past
        will need to be reset every time the
        PID needs to be used for driving straight or turning
        """

        self.PID_time_past = time.time()
        self.gyro_time_past = time.time()  # for the angle readings
        self.error_sum = 0
        self.error_past = 0

    def reset_PID_error(self):
        """
        replacement for reset_PID, does not reset the PID values.
        The error values for error integral and error past
        will need to be reset every time the
        PID needs to be used for driving straight or turning
        """

        self.PID_time_past = time.time()
        self.gyro_time_past = time.time()  # for the angle readings
        self.error_sum = 0
        self.error_past = 0

    def reset_gyro(self):
        """
        Resets the angles list and the "time past" variable
        used in the gyro angle calculation
        """
        self.angle_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.gyro_time_past = time.time()

    def reset_drive(self):
        """
        calls reset PID and reset Gyro
        """
        self.reset_PID()
        self.reset_gyro()

    def clamp(self, n, smallest, largest):
        """
        :param n: input number that you wish to clamp
        :param smallest: lowest value n can be
        :param largest:  largest value n should be
        :return:
        """
        return max(smallest, min(n, largest))

    def get_tilt(self, x_acc, y_acc, z_acc):
        """
        #:param x_acc: x acceleration in multiples of 1g = 9.8m/s^2
        #:param y_acc: y acceleration in multiples of 1g = 9.8m/s^2
        #:param z_acc: z acceleration in multiples of 1g = 9.8m/s^2
        #:return: orientation state, rotation angle x, rotation angle y , rotation angle z

        #using the acceleration values one can find the rotation angles
        #with respect to the strongest force being applied to Zumi
        #this force is usually gravity

         #   orientation state
         #-1 = unknown
         # 0 = probably falling or moving between states
         # 1 = camera straight up
         # 2 = camera facing down
         # 3 = on right side
         # 4 = on left side
         # 5 = wheels on floor
         # 6 = wheels facing up (upside down)
         # 7 = accelerating faster than 1 g
        """
        vector_acc = ((y_acc ** 2) + (x_acc ** 2) + (z_acc ** 2)) ** 0.5

        if vector_acc > 0:
            # preserves the sign x*abs(x)
            x_perc = x_acc * abs(x_acc) / vector_acc ** 2
            y_perc = y_acc * abs(y_acc) / vector_acc ** 2
            z_perc = z_acc * abs(z_acc) / vector_acc ** 2
        else:
            x_perc = 0
            y_perc = 0
            z_perc = 0

        # switch r_x and r_y to fit the rest of the orientations in my code
        r_y = -2 * 180.0 / math.pi * math.atan(x_perc)
        r_x = 2 * 180.0 / math.pi * math.atan(y_perc)
        r_z = 2 * 180.0 / math.pi * math.atan(z_perc)

        orientation_state = -1
        threshold = 0.55
        # threshold percent of gravity on that axis so 0.55 = 55%

        # MPU sometimes overshoots and
        # reads more than 1g. Not all axis are the same
        if vector_acc < 1.25:
            if x_perc > threshold:
                orientation_state = 1
            elif x_perc < -1 * threshold:
                orientation_state = 2
            elif y_perc > threshold:
                orientation_state = 3
            elif y_perc < -1 * threshold:
                orientation_state = 4
            elif z_perc > threshold:
                orientation_state = 5
            elif z_perc < -1 * threshold:
                orientation_state = 6
            else:
                orientation_state = 0
        elif vector_acc > 1.25:
            orientation_state = 7

        return orientation_state, r_x, r_y, r_z

    def get_orientation(self):
        """
        updates the gyro-accelerometer
        and calculates the orientation state depending
        on the accelerometer data.
        Works best if Zumi is not moving around
           orientation state
        -1 = unknown
         0 = probably falling or moving between states
         1 = camera straight up
         2 = camera facing down
         3 = on right side
         4 = on left side
         5 = wheels on floor
         6 = wheels facing up (upside down)
         7 = accelerating faster than 1 g
        :return orient (a number representing the state)
        """
        self.update_angles()
        orient = self.angle_list[10]

        return orient

    def get_orientation_message(self):
        """
        reads the current orientation state of the Gyro and Accelerometer
        which is a number and returns it as a read-able string.
        :return: current orientation
        """
        # returns a readable message of the orientation state
        state = self.get_orientation()
        orient_msg_list = ["unknown",
                           "face up", "face down",
                           "right side down", "left side down",
                           "upright", "upside down", "accelerating"]
        if state > 0:
            return orient_msg_list[state]
        else:
            # unknown
            return orient_msg_list[0]

    def update_angles(self):
        """
            Method that reads angular speeds and update the list of angles
        The first 3 are angles produced from the gyroscope readings
        The Acceleration angles are produced by finding the tilt with respect to gravity
        The complementary filtered angles are produced
        by combining both Gyro and Acc angles for the x and y axis.
        :return: angle_list
        """

        # update all the mpu readings
        self.mpu_list = self.mpu.read_all_MPU_data()
        self.mpu_list.append(self.mpu.get_temp())
        time_now = time.time()
        dt = (time_now - self.gyro_time_past)
        self.gyro_time_past = time_now

        x_acc = self.mpu_list[0]
        y_acc = self.mpu_list[1]
        z_acc = self.mpu_list[2]

        # In order to get the correct gyro speed multiply by gyro_multiplier
        # adjust if your Zumi does not get accurate angular speed readings
        angular_speed_x = self.mpu_list[3]
        angular_speed_y = self.mpu_list[4]
        angular_speed_z = self.mpu_list[5]

        # integrate the angle: add change in angle to previous angle
        self.angle_list[0] = self.angle_list[0] + angular_speed_x * dt
        self.angle_list[1] = self.angle_list[1] + angular_speed_y * dt
        self.angle_list[2] = self.angle_list[2] + angular_speed_z * dt

        # calculate tilt angles for roll and pitch with respect to the strongest force
        orient, r_x, r_y, r_z = self.get_tilt(x_acc, y_acc, z_acc)

        # x and y axis acc angle
        self.angle_list[3] = 180.0 / math.pi * math.atan2(y_acc, z_acc)
        self.angle_list[4] = -1 * 180.0 / math.pi * math.atan2(x_acc, z_acc)

        # complementary filter
        alpha = 0.02
        # take 98% of the gyro angle and add it to 2% of the acc angle
        # only activate complimentary filter if not facing straight up or down
        if abs(self.angle_list[4]) < 89:
            self.angle_list[5] = self.angle_list[0] * (1 - alpha) + self.angle_list[3] * alpha
        else:
            self.angle_list[5] = self.angle_list[0]

        if abs(self.angle_list[3]) < 89:
            self.angle_list[6] = self.angle_list[1] * (1 - alpha) + self.angle_list[4] * alpha
        else:
            self.angle_list[6] = self.angle_list[1]

        # rotation angles r_x,r_y,r_z with respect to strongest acceleration
        # usually [gravity]. These angles do not work below (-90) or above (+90)
        self.angle_list[7] = r_x
        self.angle_list[8] = r_y
        self.angle_list[9] = r_z
        self.angle_list[10] = orient

        return self.angle_list

    def read_x_angle(self):
        """
        updates and reads the angle from the gyroscope
        and returns only the x angle
        :return: x axis angle
        """
        angle_list = self.update_angles()

        x_angle = angle_list[0]
        x_angle = round(x_angle, 3)
        return x_angle

    def read_y_angle(self):
        """
        updates and reads the angle from the gyroscope
        and returns only the y angle
        :return: y axis angle
        """
        angle_list = self.update_angles()

        y_angle = angle_list[1]
        y_angle = round(y_angle, 3)
        return y_angle

    def read_z_angle(self):
        """
        updates and reads the angle from the gyroscope
        and returns only the z angle
        :return: z axis angle
        """
        # updates and reads only the  Z angle
        angle_list = self.update_angles()
        z_angle = angle_list[2]
        z_angle = round(z_angle, 3)
        return z_angle

    # DRIVING METHODS ---------------------------------------------------------------

    def drive_at_angle(self, max_speed, base_speed, desired_angle, k_p, k_d, k_i, min_error):
        """
        #Use this method if you wish to drive Zumi in a straight line or turn.

        #:param max_speed: This number will cap the max speed for the motors
        #:param base_speed: This is the "forward/reverse" speed Zumi will drive at.
        # set to 0 if you wish not to move forward or reverse.
        #:param desired_angle: This will be the angle Zumi will try to drive towards. Can be a positive or negative number
        #:param k_p: constant for proportional controller.
        #:param k_d: constant for derivative controller.
        #:param k_i: constant for integral controller.
        #:param min_error: the minimum error that the motors will stop trying to correct for
        #:return: error
        """

        # update the angles and grab the Z angle
        angles_now_list = self.update_angles()
        angle_z = angles_now_list[2]

        # calculate error values
        error = desired_angle - angle_z
        self.error_sum = self.error_sum + error

        error_change = error - self.error_past
        self.error_past = error

        # find change in time
        dt = time.time() - self.PID_time_past
        # if dt > 0.1:
        #     #make sure to cap the change in time 
        #     dt = 0.1
        self.PID_time_past = time.time()

        # PID components
        e_p = (k_p * error)
        e_d = (k_d * error_change / dt)
        e_i = (k_i * self.error_sum * dt)
        speed_offset = int(e_p + e_d + e_i)

        # if within the error threshold set motors to 0 speed
        if abs(error) < min_error:
            speed_offset = 0

        # input speed for all functions that use drive_at_angle are capped at +-MAX_USER_SPEED=80
        base_speed = self.clamp(int(base_speed), -1 * self.MAX_USER_SPEED, self.MAX_USER_SPEED)

        # make sure motor speeds never go above 127 or below - 127
        final_left_speed = self.clamp(base_speed - speed_offset, -1 * max_speed, max_speed)
        final_right_speed = self.clamp(base_speed + speed_offset, -1 * max_speed, max_speed)

        # send I2C msg to set motors to these values, with no acceleration
        self.control_motors(final_right_speed, final_left_speed)

        return error

    def turn(self, desired_angle, duration=1.5, max_speed=25, accuracy=1):
        """
        #:param degrees: the degrees you want to face can be positive or negative
        #:param duration: the number of seconds you want Zumi to "try" to turn
        #:param max_speed=20: maximum speed for turning
        #:param accuracy: the tolerance of +-  degrees.
        #example accuracy of 1 is desired_angle +-1 degrees
        #:return: nothing
        # uses drive_at_angle()
        """
        k_p = self.T_P
        k_i = self.T_I
        k_d = self.T_D
        # keep the max speed low for turning
        start_time = time.time()
        self.reset_PID()
        speed_now = 0
        accel_period = 1.0

        try:
            # acceleration to start motors
            # ----------------------------------------
            if duration >= accel_period:
                while time.time() - start_time < accel_period:
                    if speed_now < max_speed:
                        speed_now = speed_now + 1
                    self.drive_at_angle(abs(speed_now), 0, desired_angle, k_p, k_d, k_i, accuracy)
            elif duration < accel_period:
                while time.time() - start_time < abs(duration * 0.6):
                    if speed_now < max_speed:
                        speed_now = speed_now + 1
                    self.drive_at_angle(abs(speed_now), 0, desired_angle, k_p, k_d, k_i, accuracy)
            # ----------------------------------------
            while time.time() - start_time < abs(duration):
                self.drive_at_angle(max_speed, 0, desired_angle, k_p, k_d, k_i, accuracy)
        except KeyboardInterrupt:
            raise
        finally:
            self.stop()

    def go_straight(self, speed, desired_angle, max_speed=127, accuracy=0.8):
        """
        #:WARNING ONLY WORKS INSIDE A LOOP!
        #:calling it only once does not drive straight
        #:param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        #:param duration: number of seconds you want Zumi to try to drive forward
        #:param desired_angle: the desired angle
        #:return: nothing
        # uses drive_at_angle()
        """
        k_p = self.D_P
        k_i = self.D_I
        k_d = self.D_D
        self.drive_at_angle(max_speed, abs(speed), desired_angle, k_p, k_d, k_i, accuracy)

    def forward_step(self, speed, desired_angle, max_speed=127, accuracy=0.8):
        """
        #COPY OF GO_STRAIGHT just with different name added in 1.41
        #:WARNING ONLY WORKS INSIDE A LOOP!
        #:calling it only once does not drive straight
        #:param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        #:param duration: number of seconds you want Zumi to try to drive forward
        #:param desired_angle: the desired angle
        #:return: nothing
        # uses drive_at_angle()
        """

        k_p = self.D_P
        k_i = self.D_I
        k_d = self.D_D
        self.drive_at_angle(max_speed, abs(speed), desired_angle, k_p, k_d, k_i, accuracy)

    def go_reverse(self, speed, desired_angle, max_speed=127, accuracy=0.8):
        """
        #:WARNING ONLY WORKS INSIDE A LOOP! calling it only once does not drive reverse
        #:param speed: the reverse speed you want Zumi to drive at. Should only input a positive number
        #:param duration: number of seconds you want Zumi to try to drive forward
        #:param desired_angle: the desired angle
        #:return: nothing
        # uses drive_at_angle()
        """
        k_p = self.D_P
        k_i = self.D_I
        k_d = self.D_D
        self.drive_at_angle(max_speed, -1 * abs(speed), desired_angle, k_p, k_d, k_i, accuracy)

    def reverse_step(self, speed, desired_angle, max_speed=127, accuracy=0.8):
        """
        #COPY OF GO_REVERSE just with different name added in 1.41
        #:WARNING ONLY WORKS INSIDE A LOOP!calling it only once does not drive straight
        #:param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        #:param duration: number of seconds you want Zumi to try to drive forward
        #:param desired_angle: the desired angle
        #:return: nothing
        # uses drive_at_angle()
        """
        k_p = self.D_P
        k_i = self.D_I
        k_d = self.D_D
        self.drive_at_angle(max_speed, -1 * abs(speed), desired_angle, k_p, k_d, k_i, accuracy)

    def turn_left(self, desired_angle=90, duration=1):
        """
        :param desired_angle: The angle you wish to turn left from
        :param duration: the amount of time zumi will attempt to reach desired angle
        and stay at that angle
        :return: nothing
        """
        self.turn(self.read_z_angle() + abs(desired_angle), duration)

    def turn_right(self, desired_angle=-90, duration=1):
        """
        :param desired_angle: The angle you wish to turn left from
        :param duration: the amount of time zumi will attempt to reach desired angle
        and stay at that angle
        :return: nothing
        """
        # will turn right given an angle
        # duration is amount of time you will allow zumi to try to turn
        self.turn(self.read_z_angle() + -1 * abs(desired_angle), duration)

    def left_u_turn(self, speed=30, step=4, delay=0.02):
        """
        :param speed: the forward speed you want Zumi to drive at.
        :param step: the angle step size as it goes from (0 - 180)
        :param delay: the delay between each angle step
        :return: nothing
        """
        init_ang_z = self.read_z_angle()
        try:
            for i in range(0, 181, step):
                self.go_straight(speed, init_ang_z + i)
                time.sleep(delay)
        except KeyboardInterrupt:
            raise
        finally:
            self.stop()
        self.turn(init_ang_z + 180, 1)

    def right_u_turn(self, speed=30, step=4, delay=0.02):
        """
        :param speed: the forward speed you want Zumi to drive at.
        :param step: the angle step size as it goes from (0 - 180)
        :param delay: the delay between each angle step
        :return: nothing
        """

        init_ang_z = self.read_z_angle()
        try:
            for i in range(0, 181, step):
                self.go_straight(speed, init_ang_z - i)
                time.sleep(delay)
        except KeyboardInterrupt:
            raise
        finally:
            self.stop()
        self.turn(init_ang_z - 180, 1)

    def j_turn(self, speed=100, step=4, delay=0.005):
        """
        This method will make Zumi attempt a j-turn
        drive forward, do a loop and then reverse
        :param speed: the forward speed you want Zumi to drive at.
        :param step: the angle step size for the loop
        :param delay: the delay between each angle step
        :return: nothing
        """
        init_ang_z = self.read_z_angle()
        self.reset_PID()
        try:
            self.forward(speed, 1, init_ang_z)
            for i in range(0, 120, step):
                self.go_straight(50, init_ang_z - i)
                time.sleep(delay)
            for i in range(120, 180, step):
                self.go_reverse(50, init_ang_z - i)
                time.sleep(delay)
            self.reverse(speed, 1, init_ang_z - 180)
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            raise
        finally:
            self.stop()

    def parallel_park(self, speed=15, step=2, delay=0.01, angle=55, direction=1):
        """
        zumi will attempt to parallel park
        by reversing and then turning while reversing
        and finally drive forward
        :param speed: the forward speed of the motors
        :param step: the angle step size
        :param delay: the delay step size during the turning
        :return: nothing
        """
        self.reset_PID()
        init_ang_z = self.read_z_angle()
        try:
            for i in range(5):
                self.go_reverse(speed, init_ang_z)
            for i in range(0, angle, step):
                self.go_reverse(speed, init_ang_z + direction * i)
                time.sleep(delay)
            for i in range(0, angle, step):
                self.go_reverse(speed, init_ang_z + direction * angle - direction * i)
            for i in range(30):
                self.go_straight(speed, init_ang_z)
            self.turn(init_ang_z, duration=0.7)
        except KeyboardInterrupt:
            raise
        finally:
            self.stop()

    def circle(self, speed=30, step=4, direction=1, delay=0.02):
        """
        :param speed: the forward speed you want Zumi to drive at.
        :param step: the angle step size as it goes from (0 - 360)
        :param direction: -1 = right , +1 = left
        :param delay: the delay between each angle step
        :return: nothing
        """
        direction = self.clamp(direction, -1, 1)
        self.reset_PID()
        init_ang_z = self.read_z_angle()
        try:
            for i in range(0, 361, step):
                self.go_straight(speed, init_ang_z + direction * i)
                time.sleep(delay)
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            raise
        finally:
            self.stop()
        self.turn(init_ang_z + direction * 360, 1)

    def right_circle(self, speed=30, step=4):
        self.circle(speed, step, -1)

    def circle_right(self, speed=30, step=4):
        self.circle(speed, step, -1)

    def left_circle(self, speed=30, step=4):
        self.circle(speed, step, 1)

    def circle_left(self, speed=30, step=4):
        self.circle(speed, step, 1)

    def figure_8(self, speed=30, step=3, delay=0.02):
        """
        #:param speed: the forward speed you want Zumi to drive at.
        #:param step: the angle step size as it goes from (0 - 360)
        #:param delay: the delay between each angle step
        #:return: nothing
        """
        self.reset_PID()
        init_ang_z = self.read_z_angle()
        try:
            for i in range(0, 361, step):
                self.go_straight(speed, init_ang_z - i)
                time.sleep(delay)
            init_ang_z = self.read_z_angle()
            for i in range(0, 361, step):
                self.go_straight(speed, init_ang_z + i)
                time.sleep(delay)
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            raise
        finally:
            self.stop()

    def square(self, speed=40, seconds=1, direction=1):
        """
        # direction will need to be either -1 or 1 to change direction
        """
        direction = self.clamp(direction, -1, 1)
        starting_angle = self.read_z_angle()
        self.forward(speed, seconds, starting_angle + direction * 0)
        self.forward(speed, seconds, starting_angle + direction * 90)
        self.forward(speed, seconds, starting_angle + direction * 180)
        self.forward(speed, seconds, starting_angle + direction * 270)
        self.turn(starting_angle + direction * 360, 1)
        self.stop()

    def square_left(self, speed=40, seconds=1.0):
        direction = 1
        self.square(speed, seconds, direction)

    def square_right(self, speed=40, seconds=1.0):
        direction = -1
        self.square(speed, seconds, direction)

    def rectangle(self, speed=40, seconds=1.0, direction=1, ratio=2):
        """
        direction will need to be either -1 or 1 to change direction
        :param speed:
        :param seconds:
        :param direction:
        :param ratio:
        :return: nothing
        """
        direction = self.clamp(direction, -1, 1)
        starting_angle = self.read_z_angle()
        self.forward(speed, seconds, starting_angle + direction * 0)
        self.forward(speed, seconds * ratio, starting_angle + direction * 90)
        self.forward(speed, seconds, starting_angle + direction * 180)
        self.forward(speed, seconds * ratio, starting_angle + direction * 270)
        self.turn(starting_angle + direction * 360, 1)
        self.stop()

    def rectangle_left(self, speed=40, seconds=1.0, ratio=2):
        direction = 1
        self.rectangle(speed, seconds, direction, ratio)

    def rectangle_right(self, speed=40, seconds=1.0, ratio=2):
        direction = -1
        self.rectangle(speed, seconds, direction, ratio)

    def triangle(self, speed=40, seconds=1.5, direction=1):
        """
        direction will need to be either -1 or 1 to change direction
        :param speed:
        :param seconds:
        :param direction:
        :return: nothing
        """
        direction = self.clamp(direction, -1, 1)
        starting_angle = self.read_z_angle()
        self.forward(speed, seconds, starting_angle + direction * 0)
        self.forward(speed, seconds, starting_angle + direction * 120)
        self.forward(speed, seconds, starting_angle + direction * 240)
        self.turn(starting_angle + direction * 360, 1)
        self.stop()

    def triangle_left(self, speed=40, seconds=1.5):
        direction = 1
        self.triangle(speed, seconds, direction)

    def triangle_right(self, speed=40, seconds=1.5):
        direction = -1
        self.triangle(speed, seconds, direction)

    def calc_motor_data(self, motor, time_out=0.5, set_speed=40, graphs=False,
                        wheel_2_wheel_dist=6.3, wheel_rad=1.5):
        self.reset_gyro()

        time_list = []
        gyro_list = []
        batt_list = []

        # reset variables
        ang_speed_list = []
        init_time = time.time()
        time_passed = 0

        try:
            if motor == "LEFT":
                self.control_motors(0, set_speed)
            elif motor == "RIGHT":
                self.control_motors(set_speed, 0)
            else:
                print("please select a motor, LEFT or RIGHT")
                return -1

            while (True):
                time_passed = time.time() - init_time

                current_angle = self.update_angles()[2]
                ang_speed_list.append(self.mpu_list[5])
                time_list.append(time_passed)
                gyro_list.append(current_angle)
                batt_list.append(self.get_battery_voltage())

                if time_passed >= time_out:
                    break
                if abs(current_angle) > 359:
                    break
        finally:
            self.stop()

            wheel_circumference = 2 * math.pi * wheel_rad

            arc_length = abs(2 * math.pi * current_angle / 360 * wheel_2_wheel_dist)
            num_turns = arc_length / wheel_circumference
            rps = num_turns / time_passed
            avg_ang_speed = sum(ang_speed_list) / len(ang_speed_list)
            wheel_speed = rps * wheel_circumference
            try:
                if graphs == True:
                    import matplotlib.pyplot as plt
                    plt.plot(time_list, ang_speed_list, 'bo')
                    plt.ylabel('ang speed gyro')
                    plt.xlabel('time')
                    plt.show()

                    plt.plot(time_list, gyro_list, 'go')
                    plt.ylabel('gyro')
                    plt.xlabel('time')
                    plt.show()

                    plt.plot(time_list, batt_list, 'go')
                    plt.ylabel('battery')
                    plt.xlabel('time')
                    plt.show()
            except:
                pass

        return [rps, wheel_speed, arc_length, num_turns, avg_ang_speed]

    def calc_both_motor_data(self, time_out=2, set_speed=40, graphs=True,
                             wheel_2_wheel_dist=6.3, wheel_rad=1.5):
        self.reset_gyro()

        time_list = []
        gyro_list = []
        batt_list = []

        # reset variables
        ang_speed_list = []
        init_time = time.time()
        time_passed = 0

        try:

            self.control_motors(set_speed, set_speed)

            while (True):
                time_passed = time.time() - init_time

                current_angle = self.update_angles()[2]
                ang_speed_list.append(self.mpu_list[5])
                time_list.append(time_passed)
                gyro_list.append(current_angle)
                batt_list.append(self.get_battery_voltage())

                if time_passed >= time_out:
                    break
                if abs(current_angle) > 359:
                    break
        finally:
            self.stop()

            wheel_circumference = 2 * math.pi * wheel_rad

            arc_length = abs(2 * math.pi * current_angle / 360 * wheel_2_wheel_dist)
            num_turns = arc_length / wheel_circumference
            rps = num_turns / time_passed
            avg_ang_speed = sum(ang_speed_list) / len(ang_speed_list)
            wheel_speed = rps * wheel_circumference
            try:
                if graphs == True:
                    import matplotlib.pyplot as plt
                    plt.plot(time_list, ang_speed_list, 'bo')
                    plt.ylabel('ang speed gyro')
                    plt.xlabel('time')
                    plt.show()

                    plt.plot(time_list, gyro_list, 'go')
                    plt.ylabel('gyro')
                    plt.xlabel('time')
                    plt.show()

                    plt.plot(time_list, batt_list, 'go')
                    plt.ylabel('battery')
                    plt.xlabel('time')
                    plt.show()
            except:
                pass

        return [rps, wheel_speed, arc_length, num_turns, avg_ang_speed]

    def motor_test(self, time_out=2, set_speed=40):

        data = self.calc_motor_data("LEFT", time_out=time_out, set_speed=set_speed)
        print("LEFT MOTOR")
        print("motor ", int(data[0] * 100) / 100, "rotations per second")
        print("motor speed ", int(data[1] * 100) / 100, "cm/s")
        left_speed = int(data[1] * 100) / 100
        time.sleep(1)
        data = self.calc_motor_data("RIGHT", time_out=time_out, set_speed=set_speed)
        print("RIGHT MOTOR")
        print("motor ", int(data[0] * 100) / 100, "rotations per second")
        print("motor speed ", int(data[1] * 100) / 100, "cm/s")
        right_speed = int(data[1] * 100) / 100

        if left_speed > right_speed:
            print("Your left motor is ", round(left_speed / right_speed, 3), " faster than the right motor")
        elif left_speed < right_speed:
            print("Your right motor is ", round(left_speed / right_speed, 3), " faster than the left motor")

    def forward(self, speed=40, duration=1.0, desired_angle=None, accuracy=1.0):
        '''
        #:param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        #:param duration: number of seconds you want Zumi to try to drive forward
        #:param desired_angle: the desired angle, defaults to None 
        #       in order to drive in direction Zumi is facing.
        #:return: nothing
        '''
        # PID Values
        k_p = self.D_P
        k_i = self.D_I
        k_d = self.D_D

        self.reset_PID()

        if desired_angle is None:
            # if no input find the z angle and go in that direction
            desired_angle = self.read_z_angle()

        max_speed = 127

        start_time = time.time()
        speed_now = 0
        accel_period = 0.7
        try:
            # acceleration to start motors
            # ----------------------------------------
            if duration >= accel_period:
                while time.time() - start_time < accel_period:
                    if (speed_now < max_speed):
                        speed_now = speed_now + 1
                    self.drive_at_angle(60, abs(speed_now), desired_angle, k_p, k_d, k_i, accuracy)
            elif duration < accel_period:
                while time.time() - start_time < abs(duration * 0.6):
                    if (speed_now < max_speed):
                        speed_now = speed_now + 1
                    self.drive_at_angle(60, abs(speed_now), desired_angle, k_p, k_d, k_i, accuracy)
            # -----------------------------------------

            # stay with constant max speed
            while (time.time() - start_time) < abs(duration):
                self.drive_at_angle(max_speed, abs(speed), desired_angle, k_p, k_d, k_i, accuracy)
        except KeyboardInterrupt:
            raise
        finally:
            self.stop()

    def reverse(self, speed=40, duration=1.0, desired_angle=None, accuracy=1.0):
        '''
        #:param speed: the forward speed you want Zumi to drive at. Should only input a positive number
        #:param duration: number of seconds you want Zumi to try to drive forward
        #:param desired_angle: the desired angle, defaults to None 
        #       in order to drive reverse in direction Zumi is facing.
        #:return: nothing
        '''

        # PID Values
        k_p = self.D_P
        k_i = self.D_I
        k_d = self.D_D
        self.reset_PID()
        if desired_angle is None:
            # if no input find the z angle and go in that direction
            desired_angle = self.read_z_angle()

        max_speed = 127

        start_time = time.time()
        speed_now = 0
        accel_period = 0.7

        try:
            # ----------------------------------------
            if duration >= accel_period:
                while time.time() - start_time < accel_period:
                    if speed_now < max_speed:
                        speed_now = speed_now + 1
                    self.drive_at_angle(60, -1 * abs(speed_now), desired_angle, k_p, k_d, k_i, accuracy)
            elif duration < accel_period:
                while time.time() - start_time < abs(duration * 0.6):
                    if speed_now < max_speed:
                        speed_now = speed_now + 1
                    self.drive_at_angle(60, -1 * abs(speed_now), desired_angle, k_p, k_d, k_i, accuracy)
            # ----------------------------------------
            while (time.time() - start_time) < abs(duration):
                self.drive_at_angle(max_speed, -1 * abs(speed), desired_angle, k_p, k_d, k_i, accuracy)
        except KeyboardInterrupt:
            raise
        finally:
            self.stop()

    def smooth_turn_left(self, desired_angle=90, speed=20, step=2):
        """
        turns left gradually while going forward
        until Zumi reaches the angle desired
        relative the initial angle facing.
        You can set the forward speed.
        desired angle is in degrees.
        :param desired_angle: angle desired must be positive
        :param speed: forward speed
        :param step: rate of change of angle
        :return: nothing
        """
        self.reset_PID()
        init_ang_z = self.read_z_angle()
        try:
            for i in range(0, desired_angle, step):
                self.go_straight(speed, init_ang_z + i)
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            raise
        finally:
            self.stop()

    def smooth_turn_right(self, desired_angle=90, speed=20, step=2):
        """
        turns right gradually while going forward
        until Zumi reaches the angle desired
        relative the initial angle facing.
        You can set the forward speed.
        desired angle is in degrees.
        :param desired_angle: angle desired must be positive
        :param speed: forward speed
        :param step: rate of change of angle
        :return: nothing
        """
        self.reset_PID()
        init_ang_z = self.read_z_angle()
        try:
            for i in range(0, desired_angle, step):
                self.go_straight(speed, init_ang_z - i)
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            raise
        finally:
            self.stop()

    def smooth_forward(self, duration, rate=1):
        """
        drive gradually forward at a set rate
        for the motors to increase speed given a certain duration.
        example. if duration equals 2, then for 1 second
         the motor speed will increase to a max speed, and then for the
         last second motor speed will decrease back down to 0

        :param duration: duration in seconds for forward command
        :param rate: rate at which speed changes
        :return: nothing
        """
        self.reset_PID()
        init_ang_z = self.read_z_angle()
        time_start = time.time()
        time_passed = 0
        speed = 0
        try:
            while (duration / 2 > time_passed):
                time_passed = time.time() - time_start
                self.go_straight(speed, init_ang_z)
                if (speed < 126):
                    speed = speed + rate
                    if speed > 126:
                        speed = 126
                else:
                    speed = 126

            while (duration > time_passed):
                time_passed = time.time() - time_start
                self.go_straight(speed, init_ang_z)
                if (speed > -126):
                    speed = speed - rate
                    if speed < -126:
                        speed = -126
                else:
                    speed = -126
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            raise
        finally:
            self.stop()

    def smooth_reverse(self, duration, rate=1):
        """
        drive gradually reverse at a set rate
        for the motors to increase speed given a certain duration.
        example. if duration equals 2, then for 1 second
        the motor speed will increase to a max speed, and then for the
        last second motor speed will decrease back down to 0

        :param duration: duration in seconds for forward command
        :param rate: rate at which speed changes
        :return: nothing
        """
        self.reset_PID()
        init_ang_z = self.read_z_angle()
        time_start = time.time()
        time_passed = 0
        speed = 0
        try:
            while duration / 2 > time_passed:
                time_passed = time.time() - time_start
                self.go_reverse(speed, init_ang_z)
                if speed < 126:
                    speed = speed + rate
                    if speed > 126:
                        speed = 126
                else:
                    speed = 126

            while duration > time_passed:
                time_passed = time.time() - time_start
                self.go_reverse(speed, init_ang_z)
                if speed > -126:
                    speed = speed - rate
                    if speed < -126:
                        speed = -126
                else:
                    speed = -126
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            raise
        finally:
            self.stop()

    def forward_avoid_collision(self, speed=40, duration=1.0, desired_angle=None, left_th=150, right_th=150):
        # Zumi will drive forward at the set speed, for a set  duration, the angle can be set as well
        if desired_angle is None:
            desired_angle = self.read_z_angle()
        start_time = time.time()

        while True:
            # update ir values for front IR sensors
            ir_readings = self.get_all_IR_data()
            front_right_ir = ir_readings[0]
            front_left_ir = ir_readings[5]
            time_elapsed = time.time() - start_time

            if front_left_ir < left_th or front_right_ir < right_th:
                self.stop()
                break
            if time_elapsed > duration:
                break

            self.forward_step(speed, desired_angle)

        self.stop()

        return desired_angle, time_elapsed

    def reverse_avoid_collision(self, speed=40, duration=1.0, desired_angle=None, left_th=100, right_th=100):
        # Zumi will drive reverse at the set speed, for a set  duration, the angle can be set as well
        if desired_angle is None:
            desired_angle = self.read_z_angle()
        start_time = time.time()

        while True:
            # update ir values for front IR sensors
            ir_readings = self.get_all_IR_data()
            back_right_ir = ir_readings[2]
            back_left_ir = ir_readings[4]
            time_elapsed = time.time() - start_time

            if back_left_ir < left_th or back_right_ir < right_th:
                self.stop()
                break
            if time_elapsed > duration:
                break

            self.reverse_step(speed, desired_angle)

        self.stop()

        return desired_angle, time_elapsed

    def move_inches(self, distance, angle=None, k_p=None, k_i=None, k_d=None):
        """
        This method uses a best fit linear approximation of the
        distance traveled over time.
        y = mx + b where y is the distance traveled in inches
        m is the predicted speed in inches per second
        x is the time elapsed in seconds
        b is the slope intercept.

        if the move inches command default values
        do not work as well for you can adjust the
        following parameters in your program

        zumi.PRED_SLOPE_INT_INCH #this is the predicted speed in inches per second
        zumi.PRED_SPEED_INCHES_SEC #the slope intercept.
        zumii.PRED_SET_SPEED #when you create the best fit line what was the speed set to

        the default values are the following
        zumi.PRED_SPEED_INCHES_SEC = 6
        zumi.PRED_SLOPE_INT_INCH = 0
        zumi.PRED_SET_SPEED = 40

        :param distance: the amount of inches to travel
        :param angle: the angle to travel that distance
        :param k_p: P gain, proportional gain for driving
        :param k_i: I gain, integral gain for driving
        :param k_d: D gain, derivative gain for driving
        :return: error_list, time_list
        the error list is a collection of gyroscope offset from the
        desired angle and the time_list is the time which that
        angle error was collected

        Note: This method will not update the coordinates
        """
        self.reset_PID()
        if angle is None:
            angle = self.read_z_angle()
        if k_p is None:
            k_p = self.D_P
        if k_i is None:
            k_i = self.D_I
        if k_d is None:
            k_d = self.D_D
        # d = m*dt+b
        # solve for dt which is the duration
        # dt = (d - b)/m
        duration = (distance - self.PRED_SLOPE_INT_INCH) / self.PRED_SPEED_INCHES_SEC

        # turn for that duration
        self.reset_PID()
        self.turn(angle, duration=1.5)

        error_list = []
        time_list = []

        # if there is a distance go at speed 40 at that angle
        time_start = time.time()
        time_elapsed = 0
        self.reset_PID()
        try:
            while duration > time_elapsed:
                # update the time that has passed
                time_elapsed = time.time() - time_start
                # take a step in that direction going forward
                error = self.drive_at_angle(127, self.PRED_SET_SPEED, angle, k_p, k_d, k_i, 0)
                error_list.append(error)
                time_list.append(time_elapsed)
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            self.stop()
            raise
        finally:
            self.stop()

    def move_centimeters(self, distance, angle=None, k_p=None, k_i=None, k_d=None):
        """
        Same as move_inches but takes in a distance that is in centimeters.
        """
        distance_in_inches = distance * 0.394
        self.move_inches(distance_in_inches, angle, k_p, k_i, k_d)

    def move_to_coordinate(self, desired_x, desired_y, k_p=None, k_i=None, k_d=None, units="in"):
        """
        Zumi attempts to drive to the desired coordinate in inches from its initial location
        Zumi has to make all the move in order for this to work, you cant
        pick up zumi and expect it to update its coordinates
        :param desired_x: x position to travel to
        :param desired_y: y position to travel to
        :param k_p: p gain
        :param k_i: i gain
        :param k_d: d gain
        :param units: in or cm as a string
        :return:
        """

        if units == "in":
            # coordinate by default are in inches
            current_x = self.coordinate[0]
            current_y = self.coordinate[1]

            dx = desired_x - current_x
            dy = desired_y - current_y

        elif units == "cm":
            # coordinate by default are in inches so convert to centimeters
            current_x = self.coordinate[0] * 2.54
            current_y = self.coordinate[1] * 2.54

            # desired x and desired y will be in centimeters
            dx = desired_x - current_x
            dy = desired_y - current_y

        # find the angle we need to travel
        angle = math.degrees(math.atan2(dy, dx))

        # check current heading and rotations done
        angle = angle + int(self.angle_list[2] / 360) * 360

        ang_diff = self.angle_list[2] - angle

        # make sure to handle cases where angle is above 360 degrees
        if ang_diff > 200:
            angle = 360 - abs(angle)
        elif ang_diff < -200:
            angle = -1 * (360 - abs(angle))
        else:
            pass

        # find the distance to the coordinate
        distance = math.hypot(dx, dy)

        if units == "in":
            self.move_inches(distance, angle, k_p, k_i, k_d)
            # update the coordinates
            self.coordinate[0] = desired_x
            self.coordinate[1] = desired_y
        elif units == "cm":
            self.move_centimeters(distance, angle, k_p, k_i, k_d)
            # update the coordinates
            self.coordinate[0] = desired_x / 2.54
            self.coordinate[1] = desired_y / 2.54

    def get_x_coordinate(self, units="in"):
        """
        Gets the x coordinate which Zumi thinks it is at
        the coordinate is in inches
        can take in the units you wish to display the result
        "in" for inches or "cm" for centimeters
        1 inch = 2.54 centimeters
        """
        multiplier = 1
        if units == "in":
            multiplier = 1
        elif units == "cm":
            multiplier = 2.54

        return self.coordinate[0] * multiplier

    def get_y_coordinate(self, units="in"):
        """
        Gets the y coordinate which Zumi thinks it is at
        the coordinate is in inches
        can take in the units you wish to display the result
        "in" for inches or "cm" for centimeters
        1 inch = 2.54 centimeters
        """
        multiplier = 1
        if units == "in":
            multiplier = 1
        elif units == "cm":
            multiplier = 2.54

        return self.coordinate[1] * multiplier

    def reset_coordinate(self):
        """
        Resets the coordinates which zumi thinks it is at
        the coordinates are in inches
        """
        self.coordinate[0] = 0
        self.coordinate[1] = 0

    def drive_path(self, path_list, locations, units="in"):
        """
        #location coordinates must be in this format ["number as string", x, y]
        where x and y are in inches
        #example:
        #location_coordinates =[
        #["1", 0,0],
        #["2", 43,0],
        #["3", 0,43],
        #["4", 43,43]
        #]

        #and path_list must be this format
        #('1', '3', '4', '2')
        #Where Zumi will drive from 1 then 3 then 4 then 2
        #units is either in for inches and cm for centimeters

        """
        for coordinate in path_list:
            ind = int(coordinate)
            self.move_to_coordinate(locations[ind - 1][1], locations[ind - 1][2], units)

    def stop(self):
        """
        # Zumi stops
        # and updates the motor speeds list
        """
        try:
            self.reset_PID()
            self.motor_speeds = [0, 0, time.time() - self.start_time]
            self.bus.write_byte(Device.Arduino, _create_command(CommandType.Motor, MotorCommand.Stop, 0))
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            time.sleep(self.MIN_I2C_DELAY)
            self.stop()

    def hard_brake(self):
        """
        #Zumi stops immediately
        #Args: None
        """
        self.stop()

    def control_motors(self, right, left, acceleration=0):
        """
        #updated in 1.41 left and right param names changed to
        # right_motor and left_motor and match correct order

        #Sets the speed of each motor. The changes take place immediately;
        #if one were to call control_motors(30, 30) while
        #Zumi was stopped, Zumi would immediately drive forward
        #Args: 2 integers, from -126 to 127, for the left and right motors.
        #on the arduino side control_motors(-126,-126) will result
        #in analog write 176/255 in reverse for both motors. and control_motors(127,127) -> (177,177)
        #Acceleration parameter does nothing above 0.9 library
        #is no longer used. it will be removed in later versions
        """
        right = self.clamp(right, -126, 127)
        left = self.clamp(left, -126, 127)
        # update the list that holds the current values and time they were set
        self.motor_speeds = [right, left, time.time() - self.start_time]
        right = _convert_to_arduino_speed(right)
        left = _convert_to_arduino_speed(left)
        try:
            self.bus.write_i2c_block_data(Device.Arduino,
                                          _create_command(CommandType.Motor, MotorCommand.SetMotorsToSpeed,
                                                          acceleration), [right, left])
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            time.sleep(self.MIN_I2C_DELAY)
            self.stop()

    # ------------------------------------------------------------------------------

    def send_LED_byte(self):
        LED_byte = self.LED_user_pattern
        for i in range(5):
            if _bit_read(self.LED_signals, (4 - i)) == 1:
                j = 4 - i
                if j == 0:
                    LED_byte = 0b11110000
                elif j == 1:
                    # 1x1x 0x0x - Left turn signal
                    LED_byte = _bit_clear(LED_byte, 1)
                    LED_byte = _bit_clear(LED_byte, 3)
                    LED_byte = _bit_set(LED_byte, 5)
                    LED_byte = _bit_set(LED_byte, 7)
                elif j == 2:
                    # x1x1 x0x0 - Right turn signal
                    LED_byte = _bit_clear(LED_byte, 0)
                    LED_byte = _bit_clear(LED_byte, 2)
                    LED_byte = _bit_set(LED_byte, 4)
                    LED_byte = _bit_set(LED_byte, 6)
                elif j == 3:
                    # xx11 xx11 - Brake lights
                    LED_byte = _bit_set(LED_byte, 0)
                    LED_byte = _bit_set(LED_byte, 1)
                    LED_byte = _bit_set(LED_byte, 4)
                    LED_byte = _bit_set(LED_byte, 5)
                else:
                    # 11xx 11xx - Headlights
                    LED_byte = _bit_set(LED_byte, 2)
                    LED_byte = _bit_set(LED_byte, 3)
                    LED_byte = _bit_set(LED_byte, 6)
                    LED_byte = _bit_set(LED_byte, 7)

        try:
            self.bus.write_byte_data(Device.Arduino, 0b10000000, LED_byte)
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            time.sleep(0.05)
            self.stop()

    def get_all_sensor_data(self):
        # reads every sensor on Zumi and put it on a list 34 items.
        # unix time, arduino reading, mpu readings, angle list , compass readings
        current_time = time.time()
        self.update_angles()
        arduino_readings = self.get_all_arduino_data()

        # Convert the arduino data
        if arduino_readings[8] == 255:  # this indicates the Arduino firmware version is 1.00
            arduino_readings[6] = arduino_readings[6] / 15  # this is the battery voltage
            arduino_readings[7] = 0.00  # set the vcc to 0.00 volts we dont know what it is
            arduino_readings[8] = 1.00  # set the version to 1.00
        else:  # then the firmware may be above Arduino Zumi 1.10 firmware
            arduino_readings[6] = (arduino_readings[6] / 58) + 1  # battery voltage
            arduino_readings[7] = (arduino_readings[7] / 48) + 3  # vcc voltage
            arduino_readings[8] = round((arduino_readings[8] / 10) + 1.1, 1)  # firmware version

        arduino_readings.insert(0, current_time)  # append the time the sample was taken index 0
        comp_data = []
        if self.has_compass:  # if zumi has a compass then take compass readings
            comp_data = self.compass.get_data()
            comp_data.append(self.compass.get_bearing())

        return arduino_readings + self.mpu_list + self.angle_list + comp_data

    def get_all_arduino_data(self):
        """
        #Gets the whole message sent by the arduino board.
        """
        i2c_message_length = 10
        try:
            data = self.bus.read_i2c_block_data(Device.Arduino, 192, i2c_message_length)
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            time.sleep(self.MIN_I2C_DELAY)
            return self.get_all_arduino_data()
        return data

    def get_all_IR_data(self):
        """
        #Get the readings from all 6 IR sensors. Returns a list with a length of 6, with values between 0 and 255.
        #Args: None
        """
        try:
            # read the i2c bus for ir data
            data = self.bus.read_i2c_block_data(Device.Arduino, 192, 6)
            # save ir data to a list
            self.ir_list = data
            # delay to avoid i/o error
            time.sleep(self.MIN_I2C_DELAY)
        except IOError:
            time.sleep(self.MIN_I2C_DELAY)
            return self.get_all_IR_data()
        return data

    # added in 1.41
    def update_IR(self):
        return self.get_all_IR_data()

    def get_IR_data(self, ir_sensor_index):
        """
        #Get the reading from a single IR sensor. Returns a value between 0 and 255.
        #Args: 1 integer, from 0 to 5. Each number corresponds to a different IR sensor:

        #0 / IR.FRONT_RIGHT = Front right sensor
        #1 / IR.BOTTOM_RIGHT = Bottom right sensor
        #2 / IR.BACK_RIGHT = Back right sensor
        #3 / IR.BOTTOM_LEFT = Bottom left sensor
        #4 / IR.BACK_LEFT = Back left sensor
        #5 / IR.FRONT_LEFT = Front left sensor
        """
        try:
            if not _is_in_range(ir_sensor_index, 0, 5):
                raise ValueOutOfRangeError("IR sensor index must be a value between 0 and 5")
            data = self.get_all_IR_data()
            self.ir_list = data
        except IOError:
            return self.get_IR_data(ir_sensor_index)
        except ValueOutOfRangeError as e:
            print(e)
            raise
        return data[ir_sensor_index]

    def read_IR(self, sensor):
        """
        new method in 1.3
        lets students access the IR values
        via a string, easier for students to use
        updates all the ir readings and returns only
        the desired ir sensor reading
        :param sensor: the string name of the ir sensor
            front_left
            front_right
            back_left
            back_right
            bottom_left
            bottom_right
        :return: ir_value of specified string input
        """
        ir_value = -1
        ir_list = self.get_all_IR_data()

        if sensor == 'front_left':
            # 5 / IR.FRONT_LEFT = Front left sensor
            ir_value = ir_list[self.IR_front_left]
        elif sensor == 'front_right':
            # 0 / IR.FRONT_RIGHT = Front right sensor
            ir_value = ir_list[self.IR_front_right]
        elif sensor == 'back_left':
            # 4 / IR.BACK_LEFT = Back left sensor
            ir_value = ir_list[self.IR_back_left]
        elif sensor == 'back_right':
            # 2 / IR.BACK_RIGHT = Back right sensor
            ir_value = ir_list[self.IR_back_right]
        elif sensor == 'bottom_left':
            # 3 / IR.BOTTOM_LEFT = Bottom left sensor
            ir_value = ir_list[self.IR_bottom_left]
        elif sensor == 'bottom_right':
            # 1 / IR.BOTTOM_RIGHT = Bottom right sensor
            ir_value = ir_list[self.IR_bottom_right]
        else:
            print("please select a correct IR sensor ex: 'front_right' ")
            ir_value = -1

        return ir_value

    def find_threshold_ir(self, data):
        # takes in data in format
        # [ [ir0_0,..ir5_0], ...[ir0_n,..ir5_n]]
        # find the most common values for high and low using histograms
        n, bins = np.histogram(data, density=True)

        # sort list by frequency, most frequent will be the first 2 elements
        sorted_hist = sorted(n, reverse=True)
        th1 = -1
        th2 = -1
        for i in range(len(n)):
            # this may be one of the min or max
            # store the values
            if sorted_hist[0] == n[i]:
                th1 = bins[i]
            if sorted_hist[1] == n[i]:
                th2 = bins[i]
            # TODO Find a way to avoid error with second most common ir value being close to the max
        values = [int(th1), int(th2)]

        # sort the list in order
        values = sorted(values)

        threshold = int((th1 + th2) * 0.5)
        delta = abs(int(th2) - int(th1))
        values.append(threshold)
        values.append(delta)

        # return a list of the important values
        # order is [min, max, threshold, delta]
        return values

    def IR_calibration(self, time_out=6):
        # this program finds the min and max of all 6 ir sensors via a histogram
        # A calibration piece is necessary in order to perform this activity

        # create list for the readings to be graphed
        all_ir_list = []

        time_passed = 0
        init_time = time.time()
        self.play_note(100)

        print("Cover all IR sensors and place a white area under Zumi")
        time.sleep(1)
        while time_passed <= time_out / 2:
            all_ir_list.append(self.get_all_IR_data())
            time_passed = time.time() - init_time

        self.play_note(500)
        self.play_note(500)

        print("Uncover all IR sensors and place a black area under Zumi")
        time.sleep(1)
        while time_passed <= (time_out + 1):
            all_ir_list.append(self.get_all_IR_data())
            time_passed = time.time() - init_time

        self.play_note(100)

        th_list = []
        for i in range(6):
            val = self.find_threshold_ir(Zumi.column(i, all_ir_list))
            th_list.append(val)

        # store the threshold values make sure to overwrite previous data
        Zumi.save_data_txt(th_list,"/home/pi/ir_calibration_data.txt", append=False)

        return th_list

    # new method in 1.3
    # reads the current value and
    # determines if it is under a certain range
    # returns True or False
    def boolean_IR(self, sensor, threshold=100):
        """
        # new method in 1.3
        # reads the current value and
        # determines if it is under a certain range
        :param sensor: as a string
            front_left
        :param threshold:
        :return: True or False if IR on or off
        """
        ir_value = self.read_IR(sensor)

        if threshold > ir_value > 0:
            triggered = True
        elif threshold < ir_value < 255:
            triggered = False
        else:
            triggered = None
        return triggered

    # new method in 1.3
    def front_left_detect(self, threshold=100):
        ir_value = self.boolean_IR('front_left', threshold)
        return ir_value

    # new method in 1.3
    def front_right_detect(self, threshold=100):
        ir_value = self.boolean_IR('front_right', threshold)
        return ir_value

    # new method in 1.3
    def back_left_detect(self, threshold=100):
        ir_value = self.boolean_IR('back_left', threshold)
        return ir_value

    # new method in 1.3
    def back_right_detect(self, threshold=100):
        ir_value = self.boolean_IR('back_right', threshold)
        return ir_value

    # new method in 1.3
    def bottom_left_detect(self, threshold=100):
        ir_value = self.boolean_IR('bottom_left', threshold)
        return ir_value

    # new method in 1.3
    def bottom_right_detect(self, threshold=100):
        ir_value = self.boolean_IR('bottom_right', threshold)
        return ir_value

    # new method in 1.3
    # TODO: Rename to line_follow
    def line_follower(self, time_out, speed=10, left_thresh=None, right_thresh=None, diff_threshold=100):
        # this method will try to follow a black line on a white floor.
        # if both ir sensors detect white the Zumi will stop.
        # timeout is the amount of time you want to do line following
        # speed is base speed the motors will go forward at.
        # left thresh is the left bottom ir threshold the sensor
        # goes from 0-255 so its like a cutt off point
        # same for right thresh but for right bottom ir sensor
        # be to difference between the left and right ir sensors
        # if your Zumi shakes a lot left to right then try to make the gain smaller
        # if your zumi cant make sharp turns increase the gain by +0.1
        if left_thresh is None:
            left_thresh = self.IR_THRESHOLD_LIST[3]
        if right_thresh is None:
            right_thresh = self.IR_THRESHOLD_LIST[1]

        time_passed = 0
        init_time = time.time()
        try:
            while time_passed <= time_out:
                self.line_follow_step(speed, left_thresh, right_thresh, diff_threshold)
                # update the time passed
                time_passed = time.time() - init_time
        except KeyboardInterrupt:
            print('Caught Keyboard Interrupt')
            raise
        finally:
            # always have a stop at the end of code with control motors
            self.stop()

    # new method in 1.3
    def line_follow_step(self, speed=5, left_thresh=None, right_thresh=None, diff_threshold=100):
        if left_thresh is None:
            left_thresh = self.IR_THRESHOLD_LIST[3]
        if right_thresh is None:
            right_thresh = self.IR_THRESHOLD_LIST[1]

        ir_readings = self.get_all_IR_data()
        self.update_angles()
        left_bottom_ir = ir_readings[3]
        right_bottom_ir = ir_readings[1]

        # this is the difference between the left and right ir sensor
        # when the left value is bigger than the right then the diff will be positive
        # if the right value if bigger than the left then the diff will be negative
        diff = (left_bottom_ir - right_bottom_ir)

        if left_bottom_ir < left_thresh and right_bottom_ir < right_thresh:
            # if both ir sensors detect white then we stop
            self.stop()
        elif diff > diff_threshold:
            self.control_motors(speed, 0)
        elif diff < -diff_threshold:
            self.control_motors(0, speed)
        else:
            # if the difference is close to 0 lets just drive forward
            self.control_motors(speed, speed)

    # added in 1.54
    def line_follow_gyro_assist(self, speed=20, duration=1, angle=None, angle_adj=2, l_th=None, r_th=None):
        if l_th is None:
            l_th = self.IR_THRESHOLD_LIST[3]
        if r_th is None:
            r_th = self.IR_THRESHOLD_LIST[1]

        self.reset_PID()
        start_time = time.time()

        # if no angle is inserted, drive in the direction currently facing
        if angle is None:
            self.update_angles()
            init_ang = self.angle_list[2]
        else:
            init_ang = angle
        try:
            while time.time() - start_time < duration:
                ir_readings = self.get_all_IR_data()
                left_bottom_ir = ir_readings[3]
                right_bottom_ir = ir_readings[1]
                # if both sensors are outside stop
                if left_bottom_ir < l_th and right_bottom_ir < r_th:
                    break
                # if both sensor detect black keep going
                elif left_bottom_ir > l_th and right_bottom_ir > r_th:
                    self.go_straight(speed, init_ang)
                # if left sensor out turn to right
                elif left_bottom_ir < l_th and right_bottom_ir > r_th:
                    init_ang = init_ang - angle_adj
                    # while left_bottom_ir <100 and right_bottom_ir >100:
                    self.go_straight(speed, init_ang)
                # if right sensor out turn to left
                elif left_bottom_ir > l_th and right_bottom_ir < r_th:
                    init_ang = init_ang + angle_adj
                    # while left_bottom_ir >100 and right_bottom_ir <100:
                    self.go_straight(speed, init_ang)
                else:
                    pass
        finally:
            self.stop()

    # added in 1.54
    def funnel_align(self, speed=20, duration=1, angle=None, angle_adj=1, l_th=None, r_th=None, iteration=5):
        """
        # funnel align
        # aligns to the funnel piece in the competition field.
        # uses the bottom IR sensors to detect the black colored funnel.

        :param speed: Speed that Zumi will do this command. 0-80 integer value
        :param duration: number of seconds before the action ends.
        :param angle: The angle you wish to drive in. In degrees.
        :param angle_adj: The amount adjustment the angle will change
        if the IR sensors detect the black line out of sight
        :param r_th: right bottom IR threshold
        :param l_th: left bottom IR threshold
        :param iteration: Number of back up attempts Zumi will do if it detects white.
        :return: N/A
        """
        if l_th is None:
            l_th = self.IR_THRESHOLD_LIST[3]
        if r_th is None:
            r_th = self.IR_THRESHOLD_LIST[1]

        self.reset_PID()

        # temporarily save the default i2c delay time
        temp = self.MIN_I2C_DELAY
        # speed the IR readings, this may cause issues with i2c
        self.MIN_I2C_DELAY = temp / 2
        start_time = time.time()
        counter = 0

        if angle is None:
            self.update_angles()
            angle = self.angle_list[2]
        try:
            while time.time() - start_time < duration:

                ir_readings = self.get_all_IR_data()
                left_bottom_ir = ir_readings[3]
                right_bottom_ir = ir_readings[1]

                # if both sensors are outside, stop
                if left_bottom_ir < l_th and right_bottom_ir < r_th:
                    # increase the counter, of amount of times reversed
                    counter = counter + 1
                    self.stop()
                    time.sleep(0.5)
                    for i in range(10):
                        self.go_reverse(speed, angle)
                        ir_readings = self.get_all_IR_data()
                        left_bottom_ir = ir_readings[3]
                        right_bottom_ir = ir_readings[1]
                    self.stop()
                    time.sleep(0.5)
                    if counter is iteration:
                        break
                # if both sensor detect black keep going
                elif left_bottom_ir > l_th and right_bottom_ir > r_th:
                    self.go_straight(speed, angle)
                # if left sensor out turn to right
                elif left_bottom_ir < l_th and right_bottom_ir > r_th:
                    angle = angle - angle_adj
                    # while left_bottom_ir <100 and right_bottom_ir >100:
                    self.go_straight(speed, angle)
                # if right sensor out turn to left
                elif left_bottom_ir > l_th and right_bottom_ir < r_th:
                    angle = angle + angle_adj
                    # while left_bottom_ir >100 and right_bottom_ir <100:
                    self.go_straight(speed, angle)
                else:
                    pass
        finally:
            self.stop()
            self.MIN_I2C_DELAY = temp

    def drive_over_markers(self, road_markers=5, speed=40, ir_threshold=None, time_out=3):
        """
        # Use this method in order to drive over a certain amount of road markers.
        # The speed can be varied up to the max speed of 80 and a min of 0
        # The IR threshold will determine the min value which the IR sensor detect
        # time_out will set the number of seconds  before the loop ends.
        # the road marker are white strips that are perpendicular to Zumi's driving path
        # the markers must be above a black road in order to work.
        # Added in 1.60
        """
        if ir_threshold is None:
            ir_threshold = self.IR_THRESHOLD_LIST[3]

        # create variables for calculating derivative
        BLI_past = 0
        DBLI = 0
        markers_passed = 0
        time_passed = 0
        heading = self.read_z_angle()
        init_time = time.time()

        try:
            while True:
                self.go_straight(speed, heading)
                time_passed = time.time() - init_time
                ir_readings = self.get_all_IR_data()
                bottom_left_ir = ir_readings[3]

                if bottom_left_ir > ir_threshold:
                    BLI = 1  # 1 indicates black
                else:
                    BLI = 0  # 0 indicates white

                # Change in BLI
                DBLI = BLI - BLI_past

                if DBLI == 1 or DBLI == -1:
                    markers_passed = markers_passed + 0.5

                # update past Binary Left IR (BLI)
                BLI_past = BLI

                # program should end if timeout or number of bricks exceeded
                if time_passed > time_out or markers_passed >= road_markers:
                    break
        finally:
            self.stop()

    def get_battery_voltage(self):
        """
        #Get the reading from battery level
        #Only works in Zumi Arduino Firmware 1.00 and above
        #The code also makes sure
        #that the Arduino board is not using the I2C channel
        """
        if GPIO.input(4) == 0:  # check arduino is ok to send i2c
            try:
                data = self.get_all_arduino_data()

                # On the arduino side of things the voltage
                # measured on the battery will always be
                # below 4.2 volts, to be sent over its
                # multiplied by 15, then here divided by 15
                if data[8] == 255:  # this indicates the arduino firmware version is 1.00
                    voltage = data[6] / 15
                    voltage =  round(voltage, 3)
                else:  # then the firmware may be above Arduino Zumi 1.10 firmware
                    voltage = (data[6] / 58) + 1
                    voltage = round(voltage, 3)
                    # arduino sends over outputArray[6] = (batteryVoltage - 1) * 58;

            except IOError:
                voltage = -1
        else:
            voltage = -1

        return voltage

    def get_battery_percent(self):
        try:
            usb_voltage_detected = 1.7
            min_battery_voltage = 3.45
            max_battery_voltage = 4.14
            battery_percentage = -1
            battery_voltage = self.get_battery_voltage()

            if battery_voltage > min_battery_voltage:
                battery_percentage = (battery_voltage - min_battery_voltage) / (
                        max_battery_voltage - min_battery_voltage) * 100

            if battery_percentage > 100:
                battery_percentage = 100
            elif battery_percentage < 0:
                battery_percentage = 0
        except:
            battery_percentage = -1

        return int(battery_percentage)

    def get_vcc_voltage(self):
        """
        #Get the reading from 5v VCC level
        #Only works in Zumi Arduino Firmware 1.10 and above
        """
        try:
            data = self.get_all_arduino_data()

            if data[8] == 255:  # this indicates the arduino firmware version is 1.00
                voltage = 0.00
            else:  # then the firmware may be above Arduino Zumi 1.10 firmware
                voltage = (data[7] / 48) + 3
                voltage = round(voltage, 3)
                # zumi board sends over outputArray[7] = (checkInputVCC() - 3) * 48;
                # so we do the inverse math operation (outputArray[7]/48)+3 = InputVCC

        except IOError:
            voltage = -1

        return voltage

    def print_zumi_library_version(self):
        """
        #Get the version of zumi library pip installed
        #added in zumi.py 1.3
        """
        bash_str = "pip3 show zumi"
        msg = subprocess.check_output(bash_str, shell=True).decode()
        print(msg)

    def get_board_firmware_version(self):
        """
        #Get the Zumi Arduino Firmware version
        #works on Zumi Arduino Firmware 1.00 and above
        """

        if GPIO.input(4) == 0:  # check arduino is ok to send i2c
            try:
                data = self.get_all_arduino_data()
                if data[8] == 255:  # if the message is empty then it most likely is an old board version
                    version = 1.00
                else:
                    version = round((data[8] / 10) + 1.1, 1)
            except IOError:
                version = -1
        else:
            version = -1
        return version

    def all_lights_on(self):
        """
        Turns all lights on
        Args: None
        """
        self.LED_user_pattern = 255
        self.LED_signals = 0
        self.send_LED_byte()

    def all_lights_off(self):
        """
        Turns all lights off
        Args: None
        """
        self.LED_user_pattern = 0
        self.LED_signals = 0
        self.send_LED_byte()

    def headlights_on(self):
        """
        Turn the front lights on
        Args: None
        """
        self.LED_signals = _bit_set(self.LED_signals, 4)
        self.send_LED_byte()

    def headlights_off(self):
        """
        Turns the front lights off
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 4)
        self.send_LED_byte()

    def brake_lights_on(self):
        """
        Turn the back lights on
        Args: None
        """
        self.LED_signals = _bit_set(self.LED_signals, 3)
        self.send_LED_byte()

    def brake_lights_off(self):
        """
        Turns the back lights off
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 3)
        self.send_LED_byte()

    def hazard_lights_on(self):
        """
        Flashes all lights
        Args: None
        """
        self.LED_signals = _bit_set(self.LED_signals, 0)
        self.send_LED_byte()

    def hazard_lights_off(self):
        """
        Turns off all lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 0)
        self.send_LED_byte()

    def signal_left_on(self):
        """
        Flashes the left-front and left-back lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 2)
        self.LED_signals = _bit_set(self.LED_signals, 1)
        self.send_LED_byte()

    def signal_left_off(self):
        """
        Turns off all lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 1)
        self.send_LED_byte()

    def signal_right_on(self):
        """
        Flashes the right-front and right-back lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 1)
        self.LED_signals = _bit_set(self.LED_signals, 2)
        self.send_LED_byte()

    def signal_right_off(self):
        """
        Turns off all lights
        Args: None
        """
        self.LED_signals = _bit_clear(self.LED_signals, 2)
        self.send_LED_byte()

    def led_on(self, index):
        try:
            if not _is_in_range(index, 0, 3):
                raise ValueOutOfRangeError("Expected an index between 0 and 3, but got " + str(index) + "instead.")
            self.LED_user_pattern = _bit_set(self.LED_user_pattern, 3 - index)
            self.LED_user_pattern = _bit_set(self.LED_user_pattern, 3 - index + 4)
            self.send_LED_byte()
        except ValueOutOfRangeError as e:
            print(e)
            raise

    def led_off(self, index):
        try:
            if not _is_in_range(index, 0, 3):
                raise ValueOutOfRangeError("Expected an index between 0 and 3, but got " + str(index) + "instead.")
            self.LED_user_pattern = _bit_clear(self.LED_user_pattern, 3 - index)
            self.LED_user_pattern = _bit_clear(self.LED_user_pattern, 3 - index + 4)
            self.send_LED_byte()
        except ValueOutOfRangeError as e:
            print(e)
            raise

    def led_flash(self, index):
        try:
            if not _is_in_range(index, 0, 3):
                raise ValueOutOfRangeError("Expected an index between 0 and 3, but got " + str(index) + "instead.")
            self.LED_user_pattern = _bit_set(self.LED_user_pattern, 3 - index)
            self.LED_user_pattern = _bit_clear(self.LED_user_pattern, 3 - index + 4)
            self.send_LED_byte()
        except ValueOutOfRangeError as e:
            print(e)
            raise

    def play_note(self, note_type, note_duration=500):
        """
        Play a note, from C2 - B6.
        Args: 2 integers. The first is from 0-60, and the second is from 0 to 2500. The first integer determines the
        note being played. 0 is a rest. Any number after 0 will play a note, starting from C2, and going all
        the way up to B6. The second integer is the duration of the note, in milliseconds. While any integer in this
        range can be entered, the integer will be rounded down to a multiple of 10.
        """

        note_type = self.clamp(int(note_type), 0, 60)
        note_duration = self.clamp(int(note_duration), 0, 2500)
        # TODO Avoid error when rounded down to 0 , plays forever
        try:
            command = CommandType.Audio
            command = command << 6
            command |= note_type
            self.bus.write_byte_data(Device.Arduino, command, int(note_duration / 10))
            time.sleep(note_duration / 1000 + 0.015)
        except IOError:
            time.sleep(self.MIN_I2C_DELAY)
            self.play_note(note_type, note_duration)

        except ValueOutOfRangeError as e:
            print(e)
            raise

    # The following methods work with Zumi Arduino Firmware 1.1 and above
    def send_byte_to_arduino(self, byte):
        if self.board_firmware_version > 1.1:
            self.bus.write_byte(self.arduino_address, byte)
            time.sleep(self.MIN_I2C_DELAY)

    def z_led_on(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(193)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def z_led_off(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(194)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def sound_on(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(195)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def sound_off(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(196)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def enable_individual_led_control(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(197)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def disable_individual_led_control(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(198)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def front_left_led_on(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(199)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def front_left_led_off(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(200)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def front_right_led_on(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(201)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def front_right_led_off(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(202)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def back_left_led_on(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(203)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def back_left_led_off(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(204)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def back_right_led_on(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(205)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def back_right_led_off(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(206)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def IR_emitter_on(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(207)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def IR_emitter_off(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(208)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def IR_readings_on(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(209)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)

    def IR_readings_off(self):
        if self.board_firmware_version > 1.1:
            self.send_byte_to_arduino(210)
        else:
            print("command not supported in board firmware ", self.board_firmware_version)


if __name__ == '__main__':
    zumi = Zumi()

    print("battery voltage = ", zumi.get_battery_voltage())
    zumi.headlights_on()
    zumi.play_note(100, 100)
