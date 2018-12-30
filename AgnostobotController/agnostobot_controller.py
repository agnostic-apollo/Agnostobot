#!/usr/bin/env python

#import required libraries
import time
import RPi.GPIO as GPIO
import curses,sys
import motor_controller
import screen_controller


#ssh pi@192.168.0.190
#cd /opt/agnostobot/AgnostobotController
#python agnostobot_controller.py

robot = motor_controller.MotorController()

actions = {
	curses.KEY_UP:	robot.forward,
	curses.KEY_DOWN:  robot.backward,
	curses.KEY_LEFT:  robot.left,
	curses.KEY_RIGHT: robot.right,
	ord('='):	robot.increase_speed,
	ord('-'):	robot.decrease_speed,
	}

def Exit():
	global robot
	robot.stop_motor_controller()


def Agnostobot(screen):
	screen_controller.set_screen(screen)
	screen_controller.update_header()
	
	while True:
		curses.halfdelay(1)
		key = screen.getch()
		
		if key != -1:
			# KEY DOWN
			curses.halfdelay(3)
			action = actions.get(key)
			if action is not None:
				action()
		else:
			# KEY UP
			robot.stop()
		

if __name__ == "__main__":

	if len( sys.argv ) == 1:
		try:
			robot.start_motor_controller()
			curses.wrapper(Agnostobot)
		except KeyboardInterrupt:
			pass
		finally:
			Exit()
			print "Agnostobot Controller Stopped"

	else:
		print( "Usage: python agnostobot_controller.py")





