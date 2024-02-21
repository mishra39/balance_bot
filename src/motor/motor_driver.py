from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder, Motor
from time import sleep, time

# Set up motor control
# Motor Left
motorL_pwm = 18 #PWMOutputDevice("BOARD12")
motorL_direction_1 = 23 #DigitalOutputDevice("BOARD16")
motorL_direction_2 = 24 #DigitalOutputDevice("BOARD18")

#rotor = RotaryEncoder(16, 20, wrap=True, max_steps=) # BOARD36, BOARD38 

stdby = DigitalOutputDevice("BOARD22")

# Motor 2
motorR_pwm = PWMOutputDevice("BOARD11")
motorR_direction_1 = DigitalOutputDevice("BOARD15")
motorR_direction_2 = DigitalOutputDevice("BOARD13")

class MotorDriver:
	def __init__(self) -> None:
		self.motorL = Motor(forward=motorL_direction_1, backward=motorL_direction_2, pwm=motorL_pwm)
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
			self.motorL.forward()
			motorR_direction_1.off()
			motorR_direction_2.on()
		else:
			self.motorL.backward()
			motorR_direction_1.on()
			motorR_direction_2.off()

		# motorL_pwm.blink(spd,1-spd)
		motorR_pwm.blink(spd,1-spd)

	def motorStop(self):
		print("Stop Motor")
		stdby.off()
		self.motorL.stop()

	def __del__(self):
		print("Deleting motor")
		self.motorStop()
		motorR_pwm.close()
		motorR_direction_1.close()
		motorR_direction_2.close()

def main(args=None):
	motor_driver = MotorDriver()
	try:
		while True:
			st = time()
			motor_driver.forward(0.8)
			et = time()
			elapsed_time = et - st
			print('Execution time forward(): ', elapsed_time, 'second') 
			sleep(2)
	except KeyboardInterrupt:
		print("Keyboard Interrupt Detected")
		motor_driver.motorStop()
		del motor_driver
		


if __name__=="__main__":
	main()
