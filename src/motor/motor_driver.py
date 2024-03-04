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
encoder_counts_per_rev = 285.0 # pulses per revolution


class MotorDriver:
	def __init__(self) -> None:
		self.speed_rpm = 0.0
		self.angle_curr = 0.0
		self.prev_time = time()
		self.encoder_a = RotaryEncoder(ENCAA, ENCBA, max_steps=400) # BOARD36, BOARD38
		self.encoder_a.when_rotated = self.updateSpeed
		
	def drive(self, spd):
		print("runMotor()")
		# Clamp to [-1,1]
		if spd > 1:
			spd = 1
		elif spd < -1:
			spd = -1
		if (spd > 0):
			dir = 1
		elif (spd < 0):
			dir = -1
		
		stdby.on()
		# Drive forward
		if (dir==1):
			print("Forward Speed: " + str(spd))
			motorR_direction_1.on()
			motorR_direction_2.off()
			motorL_direction_1.on()
			motorL_direction_2.off()
			motorL_pwm.value = spd
			motorR_pwm.value = spd
		else:
			print("Reverse Speed: " + str(spd))
			motorR_direction_1.off()
			motorR_direction_2.on()
			motorL_direction_1.off()
			motorL_direction_2.on()
			motorL_pwm.value = spd
			motorR_pwm.value = spd

		motorL_pwm.blink(spd,1-spd)
		motorR_pwm.blink(spd,1-spd)

	def motorStop(self):
		print("Stop Motor")
		stdby.off()

	def printEncoder(self):
		print("Angle = %.2f" %self.angle_curr, u'\u00b0')
		print("Speed = %.2f" %self.speed_rpm + "RPM")
		
	
	def updateSpeed(self):
		#print("Encoder val = %.2f" %self.encoder_a.steps)
		curr_time = time()
		dt = curr_time - self.prev_time
		self.prev_time = curr_time
		
		# Compute motor speed
		self.angle_curr = (360.0 / encoder_counts_per_rev) * self.encoder_a.steps
		self.speed_rpm = self.angle_curr * (60.0 / dt)
		
		
		
		
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
			sleep(0.5)
			motor_driver.drive(0.8)
			motor_driver.printEncoder()
			
	except KeyboardInterrupt:
		print("Keyboard Interrupt Detected")
		motor_driver.motorStop()
		del motor_driver
		


if __name__=="__main__":
	main()
