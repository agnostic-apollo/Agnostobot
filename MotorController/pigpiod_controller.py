#!/usr/bin/env python
#import required libraries
import os, time

#file to start and stop pigpio daemon
class PIGPIO_Controller():

	def start_pigpiod(self):

		print "START: going to call STOP first."
		self.stop_pigpiod()
		print "START: STOP has been called ."

		print "START: try to START again."
		try:
			print "START: trying os.system('sudo pigpiod')"
			status = os.system('sudo pigpiod')
			print "    and the returned status is: ", status
			print "    sleeping for 1 second..."
			time.sleep(1)
			if not status:
				print "    pigpiod started successfully..."
				print "    ...because  status is Falsy"
				print "    ...and so I conclude that all is well."
			else:
				print "    pigpiod did not start successfully..."
				print "    ...because status is Truthy"
				print "    ...and so I conclude that there's been a problem"
		except Exception, e:
			print "START: There's been an exception"
			print "    and it is: ", str(e)
		print "START: I'm DONE"

	def stop_pigpiod(self):
		print "STOP: try to STOP."
		try:
			print "STOP: trying os.system('sudo killall pigpiod')"
			status = os.system('sudo killall pigpiod')  # stop it in case it's running
			print "    and the returned status is: ", status
			print "    sleeping for 1 second..."
			time.sleep(1)
			if not status:
				print "    pigpiod stopped successfully..."
				print "    ...because  status is Falsy"
				print "    ...and so I conclude that all is well."
			else:
				print "    pigpiod not stopped (probably wasn't started!)..."
				print "    ...because not status is Truthy"
				print "    ...but I still think that all is well."
		except:
			print "  OH NO! there was some exception while stopping pigpiod!"
		print "I AM STOP, and I'm DONE!"
