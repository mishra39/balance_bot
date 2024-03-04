import smbus					#import SMBus module of I2C
from time import sleep          #import
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
        
class IMU_Sensor:
    def __init__(self) -> None:
        self.imu_name = "MPU6050"
        self.raw_gyro_calib_offset = np.zeros([3,1])
        self.raw_accel_calib_offset = np.zeros([3,1])
        self.visualize = False
        self.roll_accel_old = 0.0
        self.pitch_accel_old = 0.0
        self.prev_time = 0.0
        self.curr_time = time.time()
        self.pitch_gyro_old = 0.0
        self.roll_gyro_old = 0.0
        self.MPU_Init()

    def MPU_Init(self):        
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)

        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

    def read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

    
    # Returns bias-corrected aceeleration
    def getAccelCorrected(self) -> np.ndarray:
        # Read Accelerometer raw value
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)
        
        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        accel_corrected = np.zeros([3,1])
        accel_corrected[0] = (acc_x - self.raw_accel_calib_offset[0]) / 16384.0
        accel_corrected[1] = (acc_y - self.raw_accel_calib_offset[1]) / 16384.0
        accel_corrected[2] = (acc_z - self.raw_accel_calib_offset[2]) / 16384.0
        
        return accel_corrected
    
    # Returns bias-corrected angular velocity
    def getGyroCorrected(self) -> np.ndarray:
        # Read Accelerometer raw value
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)
        
        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        gyro_corrected = np.zeros([3,1])
        gyro_corrected[0] = (gyro_x - self.raw_gyro_calib_offset[0]) / 131.0
        gyro_corrected[1] = (gyro_y - self.raw_gyro_calib_offset[1]) / 131.0
        gyro_corrected[2] = (gyro_z - self.raw_gyro_calib_offset[2]) / 131.0
        
        return gyro_corrected
        
    def printRawData(self) -> None:
        print (" Reading Data of Gyroscope and Accelerometer")
        
        #Read Accelerometer raw value
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)
        
        #Read Gyroscope raw value
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)
        
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        

        print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
    
    # print bias-corrected IMU data
    def printCorrectedImuData(self) -> None:
        print (" Corrected Data of Gyroscope and Accelerometer")
        accel_corrected = self.getAccelCorrected()
        gyro_corrected = self.getGyroCorrected()
        
        print ("Gx=%.2f" %gyro_corrected[0], u'\u00b0'+ "/s", "\tGy=%.2f" %gyro_corrected[1], u'\u00b0'+ "/s", "\tGz=%.2f" %gyro_corrected[2], u'\u00b0'+ "/s", "\tAx=%.2f g" %accel_corrected[0], "\tAy=%.2f g" %accel_corrected[1], "\tAz=%.2f g" %accel_corrected[2])
        
    def calibrateGyroscope(self, t=15.0):
        print("Calibrating gyro, make sure bot is level!")
        print("Starting gyro calibration for %.2f seconds." %n)
        gyro_x = 0.0
        gyro_y = 0.0
        gyro_z = 0.0
        
        t_start = time.time()
        t_elapsed = 0.0
        n = 0
        while (t_elapsed < t):
            gyro_x += self.read_raw_data(GYRO_XOUT_H)
            gyro_y += self.read_raw_data(GYRO_YOUT_H)
            gyro_z += self.read_raw_data(GYRO_ZOUT_H)
            t_elapsed = time.time() - t_start
            n += 1
        
        
        # compute average error
        self.gyro_offsets = np.array([gyro_x/n, gyro_y/n, gyro_z/n])

        print("Gyroscope Calibration complete!")
        print("raw gyro offsets: x = %.2f" %self.gyro_offsets[0], "\ty = %.2f" %self.gyro_offsets[1], "\tz = %.2f" %self.gyro_offsets[2])
        
    def plotCalibrationResults(self):
        fig, (uncal_gyro, cal_gyro, uncal_accel, cal_accel) = plt.subplots(2,2)
        
        # Clear all axis
        uncal_gyro.cla()
        
    def calibrateAccelerometer(self, t=15.0):
        print("Accelerometer gyro, make sure bot is level!")
        print("Starting accelerometer calibration for %.2f seconds." %n)
        accel_x = 0.0
        accel_y = 0.0
        accel_z = 0.0

        t_start = time.time()
        t_elapsed = 0.0
        n = 0
        while (t_elapsed < t):
            accel_x += self.read_raw_data(ACCEL_XOUT_H)
            accel_y += self.read_raw_data(ACCEL_YOUT_H)
            accel_z += self.read_raw_data(ACCEL_ZOUT_H)
            t_elapsed = time.time() - t_start
            n += 1
        
        
        # compute average error
        self.accel_offsets = np.array([accel_x/n, accel_y/n, accel_z/n - 16384.0])

        print("Accelerometer Calibration complete!")
        print("raw accel offsets: x = %.2f" %self.accel_offsets[0], "\ty = %.2f" %self.accel_offsets[1], "\tz = %.2f" %self.accel_offsets[2]) 

    # compute roll and pitch using gyro and accelerometer
    def calcRollandPitch(self):
        # Accelerometer
        accel = self.getAccelCorrected()
        # Calculate pitch angle (θ)
        accel_pitch = math.atan2(accel[1], math.sqrt(accel[0]**2 + accel[2]**2)) * 180.0 / np.pi # in degrees
        # Calculate roll angle (φ) 
        accel_roll = math.atan2(accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180.0 / np.pi # in degrees
        
        accel_pitch_new = 0.9 * self.pitch_accel_old + 0.1 * accel_pitch
        accel_roll_new = 0.9 * self.roll_accel_old + 0.1 * accel_roll
        self.pitch_accel_old = accel_pitch_new
        self.roll_accel_old = accel_roll_new
        
        self.prev_time = self.curr_time
        self.curr_time = time.time()
        dt = self.curr_time - self.prev_time
        
        print("dt = %.2f" %dt)
        
        # Gyroscope
        gyro = self.getGyroCorrected()
        # Calculate pitch angle (θ)
        pitch_gyro = self.pitch_gyro_old + gyro[1] * dt # deg /s * s= deg
        # Calculate roll angle (φ) 
        roll_gyro = self.roll_gyro_old + gyro[0] * dt # deg
        self.roll_gyro_old = roll_gyro
        self.pitch_gyro_old = pitch_gyro
        
        # Complementary Filter
        alpha = 0.85
        pitch_filtered = alpha * accel_pitch_new  + (1-alpha) * pitch_gyro
        roll_filtered = alpha * accel_roll_new + (1-alpha) * roll_gyro
        
        print("Pitch (θ) = %.2f" %pitch_filtered, u'\u00b0', "\tRoll (φ) = %.2f" %roll_filtered, u'\u00b0')
        

if __name__ == "__main__":
    imu_obj = IMU_Sensor()
    imu_obj.calibrateGyroscope(30.0)
    imu_obj.calibrateAccelerometer(30.0)
    if imu_obj.visualize:
            imu_obj.plot_calibration_results()
    while True:
        imu_obj.printCorrectedImuData()
        imu_obj.calcRollandPitch()
        sleep(0.1)
