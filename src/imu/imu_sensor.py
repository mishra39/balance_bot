import smbus					#import SMBus module of I2C
from time import sleep          #import
import numpy as np
import math

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

    
    # Returns aceeleration
    def getAccelCorrected(self) -> np.ndarray:
        # Read Accelerometer raw value
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)
        
        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        accel_corrected = np.zeros([3,1])
        accel_corrected[0] = acc_x - self.raw_accel_calib_offset[0] / 16384.0
        accel_corrected[1] = acc_y - self.raw_accel_calib_offset[1] / 16384.0
        accel_corrected[2] = acc_z - self.raw_accel_calib_offset[2] / 16384.0
        
        return accel_corrected
    
    # Returns angular velocity
    def getGyroCorrected(self) -> np.ndarray:
        # Read Accelerometer raw value
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)
        
        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        gyro_corrected = np.zeros([3,1])
        gyro_corrected[0] = acc_x - self.raw_gyro_calib_offset[0] / 131.0
        gyro_corrected[1] = acc_y - self.raw_gyro_calib_offset[1] / 131.0
        gyro_corrected[2] = acc_z - self.raw_gyro_calib_offset[2] / 131.0
        
        return gyro_corrected

    # compute roll and pitch using gyro and accelerometer
    def calcRollandPitch(self):
        accel = self.getAccelCorrected()
        # Calculate pitch angle (θ)
        accel_pitch = math.atan2(accel[0] / 9.8, accel[2] / 9.8) / (2 * math.pi) * 360
        # Calculate roll angle (φ)
        accel_roll = math.atan2(accel[1] / 9.8, accel[2] / 9.8) / (2 * math.pi) * 360
        
        print("Pitch (θ) = %.2f" %accel_pitch + " rad", "\tRoll (φ) = %.2f" %accel_roll + " rad")
        
        
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
    
    def calibrateGyroscope(self, n=100):
        print("Calibrating gyro, make sure bot is level!")

        gyro_x = 0.0
        gyro_y = 0.0
        gyro_z = 0.0

        # get sum of n gyro values
        for i in range(n):
            gyro_x += self.read_raw_data(GYRO_XOUT_H)
            gyro_y += self.read_raw_data(GYRO_YOUT_H)
            gyro_z += self.read_raw_data(GYRO_ZOUT_H)
        
        
        # compute average error
        self.gyro_offsets = np.array([gyro_x/n, gyro_y/n, gyro_z/n])

        print("Gyroscope Calibration complete!")
        print("raw gyro offsets: x = %.2f" %self.gyro_offsets[0], "\ty = %.2f" %self.gyro_offsets[1], "\tz = %.2f" %self.gyro_offsets[2])

    def calibrateAccelerometer(self, n=100):
        print("Accelerometer gyro, make sure bot is level!")
        accel_x = 0.0
        accel_y = 0.0
        accel_z = 0.0

        # get sum of n accel values
        for i in range(n):
            accel_x += self.read_raw_data(ACCEL_XOUT_H)
            accel_y += self.read_raw_data(ACCEL_YOUT_H)
            accel_z += self.read_raw_data(ACCEL_ZOUT_H)
        
        
        # compute average error
        self.accel_offsets = np.array([accel_x/n, accel_y/n, accel_z/n])

        print("Accelerometer Calibration complete!")
        print("raw accel offsets: x = %.2f" %self.accel_offsets[0], "\ty = %.2f" %self.accel_offsets[1], "\tz = %.2f" %self.accel_offsets[2]) 
    
              

if __name__ == "__main__":
    imu_obj = IMU_Sensor()
    imu_obj.calibrateGyroscope()
    imu_obj.calibrateAccelerometer()
    while True:
        imu_obj.calcRollandPitch()
        sleep(1)
