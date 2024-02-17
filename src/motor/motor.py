from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep, time

# Set up motor control
# Motor 1
motor1_pwm = PWMOutputDevice("BOARD12")
motor1_direction_1 = DigitalOutputDevice("BOARD16")
motor1_direction_2 = DigitalOutputDevice("BOARD18")

stdby = DigitalOutputDevice("BOARD22")

# Motor 2
motor2_pwm = PWMOutputDevice("BOARD11")
motor2_direction_1 = DigitalOutputDevice("BOARD15")
motor2_direction_2 = DigitalOutputDevice("BOARD13")


def forward(spd):
	print("Forward Speed: " + str(spd))
	runMotor(0, spd, 0)

def runMotor(motor, spd, dir):
	print("runMotor()")
	stdby.on()
	if (dir==1):
		motor1_direction_1.off()
		motor1_direction_2.on()
		motor2_direction_1.off()
		motor2_direction_2.on()
	else:
		motor1_direction_1.on()
		motor1_direction_2.off()
		motor2_direction_1.on()
		motor2_direction_2.off()

	motor1_pwm.blink(spd,1-spd)
	motor2_pwm.blink(spd,1-spd)

def motorStop():
	print("Stop Motor")
	stdby.off()

def main(args=None):
	try:
		while True:
			st = time()
			forward(0.8)
			et = time()
			elapsed_time = et - st
			print('Execution time forward(): ', elapsed_time, 'second') 
			sleep(2)
			motorStop()
			sleep(.35)
	except KeyboardInterrupt:
		print("Keyboard Interrupt Detected")
		# Motor 1 close
		motor1_pwm.close()
		motor1_direction_1.close()
		motor1_direction_2.close()
		# Motor 2 close
		motor2_pwm.close()
		motor2_direction_1.close()
		motor2_direction_2.close()


if __name__=="__main__":
	main()
