#!/usr/bin/env python
#import required libraries
import commands, pigpio, time

#class to start and stop pigpio daemon
#code taken from uhoh at https://raspberrypi.stackexchange.com/a/89086


class PigpioController():

	def __init__(self):
		self.previousStatus = 1

	def start_pigpiod(self):

		#see if it is running already
		status, process = commands.getstatusoutput('sudo pidof pigpiod')
		self.previousStatus = status #store previous status

		if status:  #it wasn't running, so start it
			print "pigpiod was not running"
			commands.getstatusoutput('sudo pigpiod')  #try to  start it
			time.sleep(0.5)
			# check it again        
			status, process = commands.getstatusoutput('sudo pidof pigpiod')

		if not status:  #if it was started successfully (or was already running)...
			pigpiod_process = process
			print "pigpiod is running, process ID is {} ".format(pigpiod_process)
			return True
			
		else:
			print "start pigpiod was unsuccessful."
			return False;

	def stop_pigpiod(self, force_kill=True):

		if not self.previousStatus and not force_kill:  #if it was already running when start_pigpiod was called and force_kill not true
			print "pigpiod was already running, so not stopping it"
			return False
		else:
			commands.getstatusoutput('sudo killall pigpiod')  #try to kill it

			time.sleep(0.5)
			#check status again        
			status, process = commands.getstatusoutput('sudo pidof pigpiod')

			if status:  #it was killed successfully
				print "pigpiod was stopped successful."
				return True
				
			else:
				print "pigpiod was not stopped successful."
				return False;

