#!/usr/bin/env python

import cv2
import sys
import rospy
import numpy as np 
from cosrap.msg import robot_parameters
from math import *

def track_color(hsv,input_frame,lower,upper):
    angle, mid_x, mid_y  = 0,0,0
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(hsv, hsv, mask = mask)
    image_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    ret, thresholded_image = cv2.threshold(image_gray, 0, 127, cv2.THRESH_BINARY)
    _, contours, hierarchy = cv2.findContours(thresholded_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        [x,y,w,h] = cv2.boundingRect(contour)
        if cv2.contourArea(contour) >200:
            if h>30:
                rectangle = cv2.minAreaRect(contour)
                enclose = cv2.boxPoints(rectangle)
                enclose = np.int0(enclose)

                coordinates = np.array(enclose)
                (x1,x2,x3) = (coordinates[1:4,0])
                (y1,y2,y3) = (coordinates[1:4,1])

                d1 = sqrt((x2-x1)**2+(y2-y1)**2)
                d2 = sqrt((x3-x2)**2+(y3-y2)**2)
                
                if d1 > d2:
                    angle = 180 - atan2((y2-y1),(x2-x1))*180/pi
                else:
                    angle = atan2((y3-y2),(x3-x2))*180/pi

                if angle > 90 and angle < 270:
                    angle = 180 - angle
                    if angle < 0:
                        angle = -angle

                mid_x = (x3+x1)/2
                mid_y = (y3+y1)/2

                cv2.drawContours(input_frame,[enclose],0,(0,0,255),2)
                    
    return input_frame, output, angle, mid_x, mid_y

if __name__ == '__main__':
    
    capture = cv2.VideoCapture(0)
    pub = rospy.Publisher("feedback", robot_parameters, queue_size=10)
    rospy.init_node('robot_position', anonymous=True)
    rate = rospy.Rate(10)
    params = robot_parameters()

    while True:
        
        _, frame = capture.read()

        frame_blue = frame
        frame_green = frame

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #Green thresholding
        lower_blue = np.array([90,50,50])
        upper_blue = np.array([130,255,255])
        blue_input=hsv
       
        blue_output, mask_b_output, blue_robot_angle, blue_mid_x, blue_mid_y = track_color(blue_input,frame_blue,lower_blue,upper_blue)
        #Green thresholding

        lower_green = np.array([30,50,120])
        upper_green = np.array([90,255,255])
        green_input=hsv

        green_output,mask_g_output, green_robot_angle, green_mid_x, green_mid_y = track_color(green_input,frame_green,lower_green,upper_green)          
        
        cv2.putText(green_output,str(green_robot_angle),(10,40),1,1,(0,255,0))
        cv2.imshow('Alpha test - Green robot', green_output)
        cv2.putText(blue_output,str(blue_robot_angle),(10,40),1,1,(0,255,0))
        cv2.imshow('Alpha test - Blue robot', blue_output)

        # print str(blue_robot_angle) + ',' + str(green_robot_angle)   

        params.blue_angle = blue_robot_angle
        params.green_angle = green_robot_angle
        params.blue_x = blue_mid_x
        params.blue_y = blue_mid_y
        params.green_x = green_mid_x
        params.green_y = green_mid_y

        if not rospy.is_shutdown():
            pub.publish(params)
            rate.sleep()
            if cv2.waitKey(1) == 27:
                break

        key = cv2.waitKey(1)

        if key == 27:
            sys.exit()
    
    capture.release()
    cv2.destroyAllWindows()
        