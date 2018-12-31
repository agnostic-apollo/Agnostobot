#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import thread
from abc import ABCMeta, abstractmethod

from math import pi as PI, degrees, radians
import os
import sys
import time

SERVO_MAX = 180
SERVO_MIN = 0

"""
   This is now an abstract base class. One subclass will implement the serial/USB transport and a second will use the SMBus.
"""
class Arduino:
    __metaclass__ = ABCMeta

    ''' Configuration Parameters
    '''
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12

    def base_init(self):
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        self.encoder_count = 0

        # Keep things thread safe
        self.mutex = thread.allocate_lock()

        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS

        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

    @abstractmethod
    def connect(self):
        ''' Connect to the Arduino port
        '''
        return

    @abstractmethod
    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        return

    @abstractmethod
    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return

    @abstractmethod
    def get_encoder_counts(self):
        ''' Get the encoder counts from the Arduino
        '''
        return

    @abstractmethod
    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return

    @abstractmethod
    def drive(self, left, right):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        return

    def drive_m_per_s(self, left, right):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(left_ticks_per_loop , right_ticks_per_loop )

    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

    @abstractmethod
    def analog_read(self, pin):
        ''' Read an analog pin
        '''
        return

    @abstractmethod
    def analog_write(self, pin, value):
        ''' Write an analog pin
        '''
        return

    @abstractmethod
    def digital_read(self, pin):
        ''' Read a digital pin
        '''
        return

    @abstractmethod
    def digital_write(self, pin, value):
        ''' Read a digital pin
        '''
        return

    @abstractmethod
    def pin_mode(self, pin, mode):
        '''Set the mode for a specific pin
        '''
        return

    @abstractmethod
    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''
        return

    @abstractmethod
    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''
        return

    @abstractmethod
    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return

    @abstractmethod
    def get_maxez1(self, triggerPin, outputPin):
        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
            sensor connected to the General Purpose I/O lines, triggerPin, and
            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
            power up, otherwise it wont range correctly for object less than 6
            inches away! The sensor reading defaults to use English units
            (inches). The sensor distance resolution is integer based. Also, the
            maxsonar trigger pin is RX, and the echo pin is PW.
        '''
        return
