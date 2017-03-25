#!/usr/bin/env python

import cv2
import sys
import numpy as np 

if __name__ == '__main__':
	
	capture = cv2.VideoCapture(0)

	while True:
		
		ret, frame = capture.read()

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		lower = np.array([30, 150, 50])
		upper = np.array([255, 255, 180])

		mask = cv2.inRange(hsv, lower, upper)
		output = cv2.bitwise_and(frame, frame, mask = mask)
		kernel = np.ones((15,15),np.float32)/255
		smoothed = cv2.filter2D(output,-1,kernel)
		image_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
		ret, thresholded_image = cv2.threshold(image_gray, 0, 127, cv2.THRESH_BINARY)
		_, contours, hierarchy = cv2.findContours(thresholded_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		# cv2.drawContours(frame, contours, -1, (0,255,0), 3)

		for contour in contours:
			[x,y,w,h] = cv2.boundingRect(contour)
			# if cv2.contourArea(contour)>3000 and cv2.contourArea(contour)<4000:
			if h>50:
				cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),1)

		cv2.imshow('frame',frame)
		cv2.imshow('mask',mask)

		key = cv2.waitKey(1)

		if key == 27:
			sys.exit()
