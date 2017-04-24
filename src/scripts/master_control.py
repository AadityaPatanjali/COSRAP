#!/usr/bin/env python

# import Robot
from math import *
import rospy
from cosrap.msg import dimensions
from cosrap.msg import robot_parameters
import cv2

max_x_range = 600
max_y_range = 440
calib_len = 200
calib_wid = 150
global init_x1
global init_y1 
global init_x2
global init_y2

def find_dimensions(data):
	total_length = data.length
	total_width = data.width

	init_x1 = 20
	init_y1 = 20

	init_x2 = ((max_x_range*total_length)/calib_len) + 20
	init_y2 = ((max_y_range*width)/calib_wid) + 20
	
	rospy.Subscriber("feedback",robot_parameters,initialize_robot)
	rospy.spin()

def initialize_robot(data):
	blue_angle = data.blue_angle
	green_angle = data.green_angle
	blue_x = data.blue_mid_x
	blue_y = data.blue_mid_y
	green_x = data.green_mid_x
	green_y = data.green_mid_y

	

if __name__ == "__main__":
	rospy.init_node('master_control',anonymous = True)
	try:
		rospy.Subscriber("dim_math",dimensions,find_dimensions)
	except rospy.ROSInterruptException:
		pass
	