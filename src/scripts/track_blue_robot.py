#!/usr/bin/env python

import cv2
import sys
import rospy
import numpy as np 
from std_msgs.msg import String
from math import *

def track_color(hsv,input_frame,lower,upper):
    x1, x2, x3, y1, y2, y3=0,0,0,0,0,0
    dx,dy = 0,0
    cart_x = []
    cart_y = []
    text = ''
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(hsv, hsv, mask = mask)
    image_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    ret, thresholded_image = cv2.threshold(image_gray, 0, 127, cv2.THRESH_BINARY)
    _, contours, hierarchy = cv2.findContours(thresholded_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        [x,y,w,h] = cv2.boundingRect(contour)
        if cv2.contourArea(contour) >70:
            if h>10:
                rectangle = cv2.minAreaRect(contour)
                enclose = cv2.boxPoints(rectangle)
                enclose = np.int0(enclose)
                for i in enclose:
                    count = 0
                    for j in i:
                        count = count%2
                        if count == 0:
                            cart_x.append(j)
                        else:
                            cart_y.append(j)
                        count = count+1
                
                i = 0

                while i<len(cart_x):
                    if i == 1:
                        x1 = cart_x[i]
                        y1 = cart_y[i]
                            
                    elif i == 2:
                        x2 = cart_x[i]
                        y2 = cart_y[i]
                    
                    elif i == 3:
                        x3 = cart_x[i]
                        y3 = cart_y[i]
                    
                    i = i+1

                d1 = sqrt((x2-x1)**2+(y2-y1)**2)
                d2 = sqrt((x3-x2)**2+(y3-y2)**2)
                
                if d1 > d2:
                    angle = atan2((y1-y2),(x2-x1))*180/pi
                else:
                    angle = atan2((y3-y2),(x3-x2))*180/pi

                text = str(int(angle))

                cv2.drawContours(input_frame,[enclose],0,(0,0,255),2)
                    
    return input_frame,output, text

if __name__ == '__main__':
    
    capture = cv2.VideoCapture(1)
    pub = rospy.Publisher('feedback', String, queue_size=10)
    rospy.init_node('robot_position', anonymous=True)
    rate = rospy.Rate(10)

    while True:
        
        _, frame = capture.read()

        frame_blue = frame
        frame_green = frame

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #Green thresholding
        lower_blue = np.array([90,50,50])
        upper_blue = np.array([130,255,255])
        blue_input=hsv
       
        blue_output, mask_b_output, blue_robot_angle = track_color(blue_input,frame_blue,lower_blue,upper_blue)
        #Green thresholding

        lower_green = np.array([30,50,120])
        upper_green = np.array([90,255,255])
        green_input=hsv

        green_output,mask_g_output, green_robot_angle = track_color(green_input,frame_green,lower_green,upper_green)          
        
        cv2.putText(green_output,str(green_robot_angle),(10,40),1,1,(0,255,0))
        cv2.imshow('Alpha test - Green robot', green_output)
        cv2.putText(blue_output,str(blue_robot_angle),(10,40),1,1,(0,255,0))
        cv2.imshow('Alpha test - Blue robot', blue_output)

        angles = blue_robot_angle + '\n' + green_robot_angle

        if rospy.is_shutdown():
            pub.publish(angles)
            rate.sleep()
            if cv2.waitKey(1) == 27:
                break

        key = cv2.waitKey(1)

        if key == 27:
            sys.exit()
    
    capture.release()
    cv2.destroyAllWindows()
        