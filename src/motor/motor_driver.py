from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder, Motor
from time import sleep, time

# Motor Pins
AIN1 = 23 # "BOARD16"
AIN2 = 24 # "BOARD18"
BIN1 = 22 # "BOARD15"
BIN2 = 27 # "BOARD13"
PWMA = 18 # "BOARD12"
PWMB = 17 # "BOARD11"
ENCAA = 16 # BOARD36
ENCBA = 20 # BOARD38

# Set up motor control
# Motor Left
motorL_pwm = PWMOutputDevice(PWMA)
motorL_direction_1 = DigitalOutputDevice(AIN1)
motorL_direction_2 = DigitalOutputDevice(AIN2)

stdby = DigitalOutputDevice(25) # "BOARD22"

# Motor 2
motorR_pwm = PWMOutputDevice(PWMB) # "BOARD11"
motorR_direction_1 = DigitalOutputDevice(BIN1) # "BOARD15"
motorR_direction_2 = DigitalOutputDevice(BIN2) # "BOARD13"

# Encoder
encoder_counts_per_rev = 780.0 # pulses per revolution
encoder_a = RotaryEncoder(ENCAA, ENCBA, max_steps=0) # BOARD36, BOARD38 

class MotorDriver:
	def __init__(self) -> None:
		pass

	def forward(self, spd):
		print("Forward Speed: " + str(spd))
		self.runMotor(spd, 0)

	def reverse(self, spd):
		print("Reverse Speed: " + str(spd))
		self.runMotor(spd, 1)

	def runMotor(self, spd, dir):
		print("runMotor()")
		stdby.on()
		if (dir==1):
			motorR_direction_1.off()
			motorR_direction_2.on()
			motorL_direction_1.on()
			motorL_direction_2.off()
		else:
			motorR_direction_1.on()
			motorR_direction_2.off()
			motorL_direction_1.off()
			motorL_direction_2.on()

		motorL_pwm.blink(spd,1-spd)
		motorR_pwm.blink(spd,1-spd)

	def motorStop(self):
		print("Stop Motor")
		stdby.off()

	def printEncoder(self):
		angle_curr = 360.0 / encoder_counts_per_rev * encoder_a.steps
		print("Angle = %.2f" %angle_curr)
		
	def __del__(self):
		print("Deleting motor")
		self.motorStop()
		motorR_pwm.close()
		motorR_direction_1.close()
		motorR_direction_2.close()
		motorL_pwm.close()
		motorL_direction_1.close()
		motorL_direction_2.close()

def main(args=None):
	motor_driver = MotorDriver()
	try:
		while True:
			sleep(0.02)
			motor_driver.forward(0.8)
			motor_driver.printEncoder()
			
	except KeyboardInterrupt:
		print("Keyboard Interrupt Detected")
		motor_driver.motorStop()
		del motor_driver
		


if __name__=="__main__":
	main()
