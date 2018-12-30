#!/usr/bin/env python

screen = None
line = 2
left_motor_speed = 0
right_motor_speed = 0

def set_screen(input_screen):
	global screen
	screen = input_screen

def print_string(text):
	global screen
	global line
	if screen != None:
		height,width = screen.getmaxyx()
		if height == line:
			screen.clear()
			update_header()
			line = 2;
		screen.addstr(line, 0, text)
		line += 1


def update_header():
	global screen
	global left_motor_speed
	global right_motor_speed

	if screen != None:
		screen.move(0,0)
		screen.clrtoeol()
		screen.move(1,0)
		screen.clrtoeol()
		screen.addstr(0, 0, "Agnostobot Controller Running")
		screen.addstr(1, 0, "left speed = " + str(left_motor_speed) + ", right speed = " + str(right_motor_speed))







