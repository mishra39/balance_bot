from imu.imu_sensor import IMU_Sensor
from motor.motor_driver import MotorDriver

import numpy as np
from time import sleep

class BalanceBot:
    def __init__(self) -> None:
        self.imu = IMU_Sensor(calibrate=False) # IMU
        self.motor_driver = MotorDriver() # Motors
        self.pitch_des_rad = 0.0 # desired pitch
        # PID
        self.PID = np.array([5.0, 0.0, 0.0])
    
    def run(self):
        # get pitch angle from IMU
        self.pitch_deg = self.imu.getPitch()
        self.pitch_rad = self.pitch_deg * (np.pi / 180.0) # calculte theta error
        self.pitch_err_rad = self.pitch_rad - self.pitch_des_rad
        print("Pitch Error: %.2f" %self.pitch_err_deg, u'\u00b0')
        print("Pitch Error: %.2f" %self.pitch_err_deg + " radians")

        motor_speed = self.PID[0] * self.pitch_err_rad # pid output for motor
        self.motor_driver.drive(motor_speed)
        print("Motor speed PID: %.2f" %motor_speed)

    def __del__(self):
        print("Deleting robot and related components")
        del self.motor_driver

def main(args=None):
    robot = BalanceBot()
    try:
        while True:
            robot.run()
            sleep(0.1)
    
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected")
        del robot


if __name__=="__main__":
    main()
