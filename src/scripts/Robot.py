#!/usr/bin/env python

# Simple two DC motor robot class.  Exposes a simple LOGO turtle-like API for
# moving a robot forward, backward, and turning.  See RobotTest.py for an
# example of using this class.
# Author: Tony DiCola
# License: MIT License https://opensource.org/licenses/MIT
import time
import atexit

from RPi import GPIO
from time import sleep

from Adafruit_MotorHAT import Adafruit_MotorHAT
M1_counter = 0
M2_counter = 0

M2_channel_ALastState = 0


class Robot(object):
    
    def __init__(self, addr=0x60, left_id=1, right_id=2, left_trim=0, right_trim=0,
                 stop_at_exit=True):
        """Create an instance of the robot.  Can specify the following optional
        parameters:
         - addr: The I2C address of the motor HAT, default is 0x60.
         - left_id: The ID of the left motor, default is 1.
         - right_id: The ID of the right motor, default is 2.
         - left_trim: Amount to offset the speed of the left motor, can be positive
                      or negative and use useful for matching the speed of both
                      motors.  Default is 0.
         - right_trim: Amount to offset the speed of the right motor (see above).
         - stop_at_exit: Boolean to indicate if the motors should stop on program
                         exit.  Default is True (highly recommended to keep this
                         value to prevent damage to the bot on program crash!).
        """
        # Initialize motor HAT and left, right motor.
        self._mh = Adafruit_MotorHAT(addr)
        self._left = self._mh.getMotor(left_id)
        self._right = self._mh.getMotor(right_id)
        self._left_trim = left_trim
        self._right_trim = right_trim
        # Start with motors turned off.
        self._left.run(Adafruit_MotorHAT.RELEASE)
        self._right.run(Adafruit_MotorHAT.RELEASE)
        # Configure all motors to stop at program exit if desired.
        if stop_at_exit:
            atexit.register(self.stop)

    def _left_speed(self, speed):
        """Set the speed of the left motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._left_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._left.setSpeed(speed)

    def _right_speed(self, speed):
        """Set the speed of the right motor, taking into account its trim offset.
        """
        assert 0 <= speed <= 255, 'Speed must be a value between 0 to 255 inclusive!'
        speed += self._right_trim
        speed = max(0, min(255, speed))  # Constrain speed to 0-255 after trimming.
        self._right.setSpeed(speed)

    def stop(self):
        """Stop all movement."""
        self._left.run(Adafruit_MotorHAT.RELEASE)
        self._right.run(Adafruit_MotorHAT.RELEASE)

    def forward(self, speed, seconds=None):
        """Move forward at the specified speed (0-255).  Will start moving
        forward and return unless a seconds value is specified, in which
        case the robot will move forward for that amount of time and then stop.
        """
        # Set motor speed and move both forward.
        self._left_speed(speed)
        self._right_speed(speed)
        self._left.run(Adafruit_MotorHAT.FORWARD)
        self._right.run(Adafruit_MotorHAT.FORWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def backward(self, speed, seconds=None):
        """Move backward at the specified speed (0-255).  Will start moving
        backward and return unless a seconds value is specified, in which
        case the robot will move backward for that amount of time and then stop.
        """
        # Set motor speed and move both backward.
        self._left_speed(speed)
        self._right_speed(speed)
        self._left.run(Adafruit_MotorHAT.BACKWARD)
        self._right.run(Adafruit_MotorHAT.BACKWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def right(self, speed, seconds=None):
        """Spin to the right at the specified speed.  Will start spinning and
        return unless a seconds value is specified, in which case the robot will
        spin for that amount of time and then stop.
        """
        # Set motor speed and move both forward.
        self._left_speed(speed)
        self._right_speed(speed)
        self._left.run(Adafruit_MotorHAT.FORWARD)
        self._right.run(Adafruit_MotorHAT.BACKWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def left(self, speed, seconds=None):
        """Spin to the left at the specified speed.  Will start spinning and
        return unless a seconds value is specified, in which case the robot will
        spin for that amount of time and then stop.
        """
        # Set motor speed and move both forward.
        self._left_speed(speed)
        self._right_speed(speed)
        self._left.run(Adafruit_MotorHAT.BACKWARD)
        self._right.run(Adafruit_MotorHAT.FORWARD)
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def left_motor(self, speed,seconds=None):
        #print " left motor comanded" , speed
        """Spin to the left motor at the specified speed.  Will start spinning and
        return unless a seconds value is specified, in which case the robot will
        spin for that amount of time and then stop.
        """
        # Set motor speed.
        motorVolt=0
        if (speed > 255):
            speed=255
        if (speed < -255):
            speed=-255
        if (speed >= 0):
            motorVolt=abs(speed)
            self._left.run(Adafruit_MotorHAT.FORWARD)
        if (speed < 0):
            motorVolt=abs(speed)
            self._left.run(Adafruit_MotorHAT.BACKWARD)
        
        self._left_speed(abs(motorVolt))
        
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def right_motor(self, speed,seconds=None):
        #print " right motor comanded",speed 
        """Spin to the left motor at the specified speed.  Will start spinning and
        return unless a seconds value is specified, in which case the robot will
        spin for that amount of time and then stop.
        """
        # Set motor speed.
        motorVolt=0
        if (speed > 255):
            speed=255
        if (speed < -255):
            speed=-255
        if (speed >= 0):
            motorVolt=abs(speed)
            self._right.run(Adafruit_MotorHAT.FORWARD)
        if (speed < 0):
            motorVolt=abs(speed)
            self._right.run(Adafruit_MotorHAT.BACKWARD)
        
        self._right_speed(abs(motorVolt))
        
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def right_encoder(self, M1_channel_A,M1_channel_B):
        print "right enco_Loop,", M1_channel_A,",",M1_channel_B,
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(M1_channel_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(M1_channel_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        global M1_counter
        M1_channel_ALastState = GPIO.input(M1_channel_A)

        try:

                while True:
                        M1_channel_AState = GPIO.input(M1_channel_A)
                        M1_channel_BState = GPIO.input(M1_channel_B)
                        if M1_channel_AState != M1_channel_ALastState:
                            if M1_channel_BState != M1_channel_AState:
                                    M1_counter += 1
                            else:
                                    M1_counter -= 1
                            print "right Encoder count ",M1_counter,
                        M1_channel_ALastState = M1_channel_AState
                        sleep(0.001)
                        return M1_counter

        finally:
                GPIO.cleanup()

    def left_encoder(self, M2_channel_A,M2_channel_B):
        #print "left enco_Loop,", M2_channel_A,",",M2_channel_B
        global M2_counter
        global M2_channel_ALastState
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(M2_channel_A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(M2_channel_B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        

        try:

                M2_channel_AState = GPIO.input(M2_channel_A)
                M2_channel_BState = GPIO.input(M2_channel_B)
                #print M2_channel_AState,",",M2_channel_ALastState
                if M2_channel_AState != M2_channel_ALastState:
                    #print "left Encoder count ",M1_counter
                    if M2_channel_BState != M2_channel_AState:
                            M2_counter += 1
                    else:
                            M2_counter -= 1
                    print "left Encoder count ",M2_counter
                M2_channel_ALastState = M2_channel_AState
                return M2_counter
                

        finally:
                GPIO.cleanup()



