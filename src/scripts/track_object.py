 #!/usr/bin/env python

import cv2
import sys
import numpy as np 
import math as m

if __name__ == '__main__':
    
    capture = cv2.VideoCapture(1)
    mid_x = 0
    mid_y = 0
    mid_dx = 0
    mid_dy = 0
    turned_angle = 0
    accum_theta = 0
    count = 1

    while True:
        
        _, frame = capture.read()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array([150 ,95, 112])
        upper = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask = mask)
        image_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        ret, thresholded_image = cv2.threshold(image_gray, 0, 127, cv2.THRESH_BINARY)
        _, contours, hierarchy = cv2.findContours(thresholded_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            [x,y,w,h] = cv2.boundingRect(contour)
            if cv2.contourArea(contour) >70 and cv2.contourArea(contour) <250:
                if h>10:
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),1)
                    mid_x = x+w/2
                    mid_y = y+h/2  

                    mid_dx = mid_dx - mid_x
                    mid_dy = mid_dy - mid_y
                        
                    x_val = str(mid_dx)
                    y_val = str(mid_dy)
                    theta = m.atan2(mid_y,mid_x)*180/m.pi
                    
                    if mid_dx != 0 and mid_dy != 0:
                        accum_theta = accum_theta+theta
                    else:
                        turned_angle = accum_theta

                    text = x_val+','+y_val
                    print text,',',theta,',',turned_angle 
                    cv2.putText(frame,text,(x-10,y-10),2,1,(0,0,255))

                    mid_dx = mid_x
                    mid_dy = mid_y
            
        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)
        
        key = cv2.waitKey(1)

        if key == 27:
            sys.exit()
            cv2.destroyAllWindows()
        
