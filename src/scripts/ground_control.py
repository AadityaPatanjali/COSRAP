import rospy
from math import pi as pi
from Robot import *
from encoder import *
import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM) 

#Motor PID Control Constants
kpRightMotor = 10
kiRightMotor = 0
kdRightMotor = 0.3

kpLeftMotor =10
kiLeftMotor =0
kdLeftMotor= 0.3

#PID parameters

left_oldMotorPIDEncoderCount = 0
left_oldSpeedError = 0
left_feedback = 0

right_oldMotorPIDEncoderCount = 0
right_oldSpeedError = 0
right_feedback = 0

left_count=0
prev_left_count = 0
right_count=0
prev_right_count = 0

leftPIDMotorsTimeStart=0
leftMotorVolt=0
rightPIDMotorsTimeStart=0
RightMotorVolt=0

#Motor  encoder connections
leftEncodeA  = 22 # Left motor channel A
leftEncodeB  = 23 # Left motor channel B
rightEncodeA = 17# Right motor channel A
rightEncodeB = 18# Right motor channel B

#Motor parameters
encoderCountsPerRotation = 12
motorGearRatio = 51.45 # The gearing ratio of the drive motor being used.
wheelDiameter = 3.2 # Wheel Diameter in cm.
axelLength = 12.9   # Axel length in cm.
robot = Robot()

GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)  

GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def main():

    # add the code that wil take the message feom the camera node and would give it to the individual motor velocities

    #Pass the notor values in cm/sec
    LeftDesVel = 10
    RightDesVel = 10

    while(1):
        # call the PID function to rotate the left motor at a desired velocity and return teh PWM value
        left_motor_values = PID_left_motor(LeftDesVel,leftEncodeA,leftEncodeB)

        # write the PWM value to the left motor
        robot.left_motor(left_motor_values)
        
        # call the PID function to rotate the right motor at a desired velocity and return teh PWM value
        right_motor_values = PID_right_motor(RightDesVel,rightEncodeA,rightEncodeB)
        # write the PWM value to the right motor

        robot.right_motor(right_motor_values) 
        #print "Right motor_values",",", right_motor_values

def PID_left_motor(desVel,EncodeA,EncodeB):

    # Keeps the rotational speeds of the individual motors at setpoints desVel (rad/s)
    global left_count
    global prev_left_count
    global left_oldSpeedError
    global left_oldMotorPIDEncoderCount
    global leftPIDMotorsTimeStart
    global leftMotorVolt
    
    timeStep = .5
    if (time.time() - leftPIDMotorsTimeStart >= timeStep):
        #Time step for controller to work on (s).
        PIDTimeStep = (time.time() - leftPIDMotorsTimeStart)

        #Call the encoder function to take left motor encoder counts
        left_count = left_encoder(10)

        #calculate the velocity feedback
        left_feedback = (wheelDiameter/2)*2.0 * pi * (left_count - left_oldMotorPIDEncoderCount) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep )

        # PID Error on left motors for vel control
        error = desVel - left_feedback
        
        #integral += error 
        diff = (error - left_oldSpeedError)

        # print "Left error  " , "," , error , "Left feedback" , "," , left_feedback,

        #recording previous error
        left_oldSpeedError = error 
        left_oldMotorPIDEncoderCount = left_count

        #PID equation
        leftMotorVolt = leftMotorVolt+(kpLeftMotor*error + kdLeftMotor*diff)

        leftMotorVolt = int(leftMotorVolt)
        leftPIDMotorsTimeStart = time.time()
        
    return leftMotorVolt

def PID_right_motor(desVel,EncodeA,EncodeB):

    # Keeps the rotational speeds of the individual motors at setpoints desVel (rad/s)
    
    global right_count
    global prev_right_count
    global right_count
    global right_oldSpeedError
    global right_oldMotorPIDEncoderCount
    global rightPIDMotorsTimeStart
    global RightMotorVolt

    
    timeStep = .5
    if (time.time() - rightPIDMotorsTimeStart >= timeStep):
        #Time step for controller to work on (s).
        PIDTimeStep = (time.time() - rightPIDMotorsTimeStart)
    
        #Call the encoder function to take left motor encoder counts       
        right_count = right_encoder(10)

        #calculate the velocity feedback
        right_feedback = (wheelDiameter/2)*2*pi*(right_count - right_oldMotorPIDEncoderCount) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep)
                
        # PID Error on left motors for vel control
        error = desVel - right_feedback
        
        #integral += error 
        
        diff = (error - right_oldSpeedError)/ PIDTimeStep

        # print "Right error" , "," , error , "Right feedback" , "," , right_feedback

        #recording previous error
        right_oldSpeedError = error
        right_oldMotorPIDEncoderCount = right_count

        #PID equation
        RightMotorVolt = RightMotorVolt+(kpRightMotor*error + kdRightMotor*diff)
        
        RightMotorVolt = int(RightMotorVolt)
        rightPIDMotorsTimeStart = time.time()

    return RightMotorVolt

GPIO.add_event_detect(22, GPIO.FALLING, callback=right_encoder)  
GPIO.add_event_detect(23, GPIO.FALLING, callback=right_encoder)
GPIO.add_event_detect(17, GPIO.FALLING, callback=left_encoder)  
GPIO.add_event_detect(18, GPIO.FALLING, callback=left_encoder)

if __name__ =='__main__':

	try:
#		rospy.loginfo('executing main')
		main()
	except rospy.ROSInterruptException:
#		rospy.loginfo('not main')
		pass
