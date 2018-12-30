#!/usr/bin/env python

#import required libraries
import time
import RPi.GPIO as GPIO
import pigpio
import pigpiod_controller
import screen_controller

#ssh pi@192.168.0.190
#cd /opt/agnostobot/MotorController
#python motor_controller.py


class MotorController():

	#pins are BCM numbering

	LEFT_MOTOR_IN_A = 9
	LEFT_MOTOR_IN_B = 10
	LEFT_MOTOR_PWM = 18

	RIGHT_MOTOR_IN_A = 14
	RIGHT_MOTOR_IN_B = 15
	RIGHT_MOTOR_PWM = 19

	MOTOR_PWM_FREQUENCY = 20000 #hz

	HIGH = GPIO.HIGH
	LOW = GPIO.LOW

	pi = None
	pwm = None
	pigpiod_controller = None

	left_motor_speed = 0
	right_motor_speed = 0


	def forward(self):
		#set motor direction
		GPIO.output(self.LEFT_MOTOR_IN_A, self.HIGH)
		GPIO.output(self.LEFT_MOTOR_IN_B, self.LOW)
		GPIO.output(self.RIGHT_MOTOR_IN_A, self.HIGH)
		GPIO.output(self.RIGHT_MOTOR_IN_B, self.LOW)

	def backward(self):
		#set motor direction
		GPIO.output(self.LEFT_MOTOR_IN_A, self.LOW)
		GPIO.output(self.LEFT_MOTOR_IN_B, self.HIGH)
		GPIO.output(self.RIGHT_MOTOR_IN_A, self.LOW)
		GPIO.output(self.RIGHT_MOTOR_IN_B, self.HIGH)
		
	def left(self):
		#set motor direction
		GPIO.output(self.LEFT_MOTOR_IN_A, self.LOW)
		GPIO.output(self.LEFT_MOTOR_IN_B, self.LOW)
		GPIO.output(self.RIGHT_MOTOR_IN_A, self.HIGH)
		GPIO.output(self.RIGHT_MOTOR_IN_B, self.LOW)

	def right(self):
		#set motor direction
		GPIO.output(self.LEFT_MOTOR_IN_A, self.HIGH)
		GPIO.output(self.LEFT_MOTOR_IN_B, self.LOW)
		GPIO.output(self.RIGHT_MOTOR_IN_A, self.LOW)
		GPIO.output(self.RIGHT_MOTOR_IN_B, self.LOW)

	def stop(self):
		#set motor direction
		GPIO.output(self.LEFT_MOTOR_IN_A, self.LOW)
		GPIO.output(self.LEFT_MOTOR_IN_B, self.LOW)
		GPIO.output(self.RIGHT_MOTOR_IN_A, self.LOW)
		GPIO.output(self.RIGHT_MOTOR_IN_B, self.LOW)


	def increase_speed(self):
		self.left_motor_speed += 7
		self.right_motor_speed += 7

		if self.left_motor_speed > 100:
			self.left_motor_speed = 100

		if self.right_motor_speed > 100:
			self.right_motor_speed = 100

		self.set_speed(self.LEFT_MOTOR_PWM, self.left_motor_speed) 
		self.set_speed(self.RIGHT_MOTOR_PWM, self.right_motor_speed) 

		screen_controller.left_motor_speed = self.left_motor_speed
		screen_controller.right_motor_speed = self.right_motor_speed
		screen_controller.update_header()


	def decrease_speed(self):
		self.left_motor_speed -= 7
		self.right_motor_speed -= 7

		if self.left_motor_speed < 0:
			self.left_motor_speed = 0

		if self.right_motor_speed < 0:
			self.right_motor_speed = 0

		self.set_speed(self.LEFT_MOTOR_PWM, self.left_motor_speed) 
		self.set_speed(self.RIGHT_MOTOR_PWM, self.right_motor_speed) 

		screen_controller.left_motor_speed = self.left_motor_speed
		screen_controller.right_motor_speed = self.right_motor_speed
		screen_controller.update_header()

	def set_speed(self, pin, speed):
		self.pi.set_PWM_dutycycle(pin, speed)
		screen_controller.print_string(str(speed))


	def start_motor_controller(self):

		#declare GPIO settings
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)

		# set up GPIO pins as OUTPUT
		GPIO.setup(self.LEFT_MOTOR_IN_A, GPIO.OUT) 
		GPIO.setup(self.LEFT_MOTOR_IN_B, GPIO.OUT)
		GPIO.setup(self.RIGHT_MOTOR_IN_A, GPIO.OUT)
		GPIO.setup(self.RIGHT_MOTOR_IN_B, GPIO.OUT)

		self.stop()

		#initialize pigpio
		self.pigpiod_controller = pigpiod_controller.PigpioController()
		started = self.pigpiod_controller.start_pigpiod()
		
		time.sleep(1)

		if not started:
			return False

		#get pi object
		self.pi = pigpio.pi()

		if not self.pi.connected:
			return False

		self.pi.set_mode(self.LEFT_MOTOR_PWM, pigpio.OUTPUT)
		self.pi.set_mode(self.RIGHT_MOTOR_PWM, pigpio.OUTPUT)

		self.pi.set_PWM_frequency(self.LEFT_MOTOR_PWM, self.MOTOR_PWM_FREQUENCY)
		self.pi.set_PWM_frequency(self.RIGHT_MOTOR_PWM, self.MOTOR_PWM_FREQUENCY)

		self.pi.set_PWM_range(self.LEFT_MOTOR_PWM, 100) #set speed range from 0-100
		self.pi.set_PWM_range(self.RIGHT_MOTOR_PWM, 100) #set speed range from 0-100


		#set default speed of left motor to 50%
		self.left_motor_speed = 50
		self.set_speed(self.LEFT_MOTOR_PWM, self.left_motor_speed) 

		#set default speed of right motor to 50%
		self.right_motor_speed = 50
		self.set_speed(self.RIGHT_MOTOR_PWM, self.right_motor_speed) 

		screen_controller.left_motor_speed = self.left_motor_speed
		screen_controller.right_motor_speed = self.right_motor_speed
		screen_controller.update_header()


	def stop_motor_controller(self):

		self.stop()
		if self.pi != None:
			self.pi.stop()
		if self.pigpiod_controller != None:
			self.pigpiod_controller.stop_pigpiod(True)
		GPIO.cleanup()



