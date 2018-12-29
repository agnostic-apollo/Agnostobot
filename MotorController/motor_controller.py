#!/usr/bin/env python

#import required libraries
import time
import RPi.GPIO as GPIO
import curses,sys
import pigpio
from pigpiod_controller import *

#ssh pi@192.168.0.190
#cd /opt/agnostobot/MotorController
#python motor_controller.py



leftMotorIN_A = 9
leftMotorIN_B = 10
leftMotorPWM = 18

rightMotorIN_A = 14
rightMotorIN_B = 15
rightMotorPWM = 19

pwmFrequency = 20000

HIGH = GPIO.HIGH
LOW = GPIO.LOW

pi = None
pwm = None
pigpiod_controller = None

leftMotorSpeed = 0
rightMotorSpeed = 0

curses_window = None
line = 0

def MoveForward():
	#set motor direction
	GPIO.output(leftMotorIN_A, HIGH)
	GPIO.output(leftMotorIN_B, LOW)
	GPIO.output(rightMotorIN_A, HIGH)
	GPIO.output(rightMotorIN_B, LOW)

def MoveBack():
	#set motor direction
	GPIO.output(leftMotorIN_A, LOW)
	GPIO.output(leftMotorIN_B, HIGH)
	GPIO.output(rightMotorIN_A, LOW)
	GPIO.output(rightMotorIN_B, HIGH)
	
def MoveLeft():
	#set motor direction
	GPIO.output(leftMotorIN_A, LOW)
	GPIO.output(leftMotorIN_B, LOW)
	GPIO.output(rightMotorIN_A, HIGH)
	GPIO.output(rightMotorIN_B, LOW)

def MoveRight():
	#set motor direction
	GPIO.output(leftMotorIN_A, HIGH)
	GPIO.output(leftMotorIN_B, LOW)
	GPIO.output(rightMotorIN_A, LOW)
	GPIO.output(rightMotorIN_B, LOW)

def Stop():
	#set motor direction
	GPIO.output(leftMotorIN_A, LOW)
	GPIO.output(leftMotorIN_B, LOW)
	GPIO.output(rightMotorIN_A, LOW)
	GPIO.output(rightMotorIN_B, LOW)


def IncreaseSpeed():
	global leftMotorSpeed
	global rightMotorSpeed

	leftMotorSpeed += 7
	rightMotorSpeed += 7

	if leftMotorSpeed > 100:
		leftMotorSpeed = 100

	if rightMotorSpeed > 100:
		rightMotorSpeed = 100

	curses_window.addstr(1, 0, "left speed = " + str(leftMotorSpeed))
	curses_window.addstr(2, 0, "right speed = " + str(rightMotorSpeed))

	#printString("left speed = " + str(leftMotorSpeed))
	#printString("right speed = " + str(rightMotorSpeed))

	SetSpeed(leftMotorPWM, leftMotorSpeed) 
	SetSpeed(rightMotorPWM, rightMotorSpeed) 


def DecreaseSpeed():
	global leftMotorSpeed
	global rightMotorSpeed

	leftMotorSpeed -= 7
	rightMotorSpeed -= 7

	if leftMotorSpeed < 0:
		leftMotorSpeed = 0

	if rightMotorSpeed < 0:
		rightMotorSpeed = 0

	curses_window.move(1,0)
	curses_window.clrtoeol()
	curses_window.addstr(1, 0, "left speed = " + str(leftMotorSpeed))
	curses_window.move(2,0)
	curses_window.clrtoeol()
	curses_window.addstr(2, 0, "right speed = " + str(rightMotorSpeed))

	#printString("left speed = " + str(leftMotorSpeed))
	#printString("right speed = " + str(rightMotorSpeed))

	SetSpeed(leftMotorPWM, leftMotorSpeed) 
	SetSpeed(rightMotorPWM, rightMotorSpeed) 


def SetSpeed(pin, speed):
	global pi
	pi.set_PWM_dutycycle(pin, speed)
	#printString(str(speed))


def printString(text):
	global curses_window
	global line
	if curses_window != None:
		height,width = curses_window.getmaxyx()
		if height == line:
			curses_window.clear()
			line = 0;
		curses_window.addstr(line, 0, text)
		line += 1

def InitializeMotorController():
	global pi
	global pwm
	global leftMotorSpeed
	global rightMotorSpeed
	global pigpiod_controller

	#declare GPIO settings
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)

	# set up GPIO pins as OUTPUT
	GPIO.setup(leftMotorIN_A, GPIO.OUT) 
	GPIO.setup(leftMotorIN_B, GPIO.OUT)
	#GPIO.setup(leftMotorPWM, GPIO.OUT)
	GPIO.setup(rightMotorIN_A, GPIO.OUT)
	GPIO.setup(rightMotorIN_B, GPIO.OUT)
	#GPIO.setup(rightMotorPWM, GPIO.OUT)

	Stop();

	#initialize pigpio
	pigpiod_controller = PIGPIO_Controller()
	pigpiod_controller.start_pigpiod()

	#get pi object
	pi = pigpio.pi()

	if not pi.connected:
		exit(0)

	pi.set_mode(leftMotorPWM, pigpio.OUTPUT)
	pi.set_mode(rightMotorPWM, pigpio.OUTPUT)

	pi.set_PWM_frequency(leftMotorPWM, pwmFrequency)
	pi.set_PWM_frequency(rightMotorPWM, pwmFrequency)

	pi.set_PWM_range(leftMotorPWM, 100)
	pi.set_PWM_range(rightMotorPWM, 100)


	#set default speed of left motor to 50%
	leftMotorSpeed = 50
	SetSpeed(leftMotorPWM, leftMotorSpeed) 

	#set default speed of right motor to 50%
	rightMotorSpeed = 50
	SetSpeed(rightMotorPWM, rightMotorSpeed) 



def Exit():
	global pi
	global pwm
	global pigpiod_controller
	
	Stop()
	if pi != None:
		pi.stop()
	if pigpiod_controller != None:
		pigpiod_controller.stop_pigpiod()
	GPIO.cleanup()

actions = {
	curses.KEY_UP:MoveForward,
	curses.KEY_DOWN:  MoveBack,
	curses.KEY_LEFT:  MoveLeft,
	curses.KEY_RIGHT: MoveRight,
	ord('='):	IncreaseSpeed,
	ord('-'):	DecreaseSpeed,
	}


def MotorController(window):
	global curses_window
	curses_window = window
	window.scrollok(1)

	printString("Motion Controller Started")
	
	while True:
		curses.halfdelay(1)
		key = window.getch()
		
		if key != -1:
			# KEY DOWN
			curses.halfdelay(3)
			action = actions.get(key)
			if action is not None:
				action()
		else:
			# KEY UP
			Stop()
		

if __name__ == "__main__":

	if len( sys.argv ) == 1:
		try:
			InitializeMotorController()
			curses.wrapper(MotorController)
		except KeyboardInterrupt:
			pass
		finally:
			Exit()
			print "Motion Controller Stopped"

	else:
		print( "Usage: python motor_controller.py")





