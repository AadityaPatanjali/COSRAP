#!/usr/bin/env python

import cv2
import numpy as np
import operator
import rospy
from OcrDigits import Digits
from cosrap.msg import dimensions as dim
from os.path import expanduser
HOME = expanduser("~")
FOLDER = HOME + '/catkin_ws/src/cosrap/src/scripts/'

def dispatch_dimensions(digit_list):
	pub = rospy.Publisher('dim_math', dim, queue_size=10)
	rospy.init_node('camera_capture', anonymous=True)
	rate = rospy.Rate(10)
	dimensions = ''
	digit_dict = {}
	total_list = []
	sub_list = []
	y_val = 0
	sorted_flag = False
	history = 0
	length = 0
	width = 0
	valid = False
	weight = 6
	leeway = 7

	# write sorting code
	for digits in digit_list:
		digit_dict = {'x' : digits.x_column, 'y' : digits.y_row, 'w' : digits.width, 'h' : digits.height, 'd' : digits.digit}
		total_list.append(digit_dict)

	total_list_sort = total_list
	total_list_sort.sort(key=operator.itemgetter('y','x','w'))

	print total_list_sort

	for proton in total_list_sort:
		count = 0
		for neutron in total_list_sort:
			if count == 0:
				y_val = proton['y']
			else:
				if y_val + leeway >= neutron['y'] and y_val - leeway <= neutron['y']:
					neutron['y'] = y_val
				else:
					continue
			count = count + 1

	print total_list_sort

	total_list_sort_x = total_list_sort
	total_list_sort_x.sort(key=operator.itemgetter('x'))
	print total_list_sort_x

	#string stitching
	for proton in total_list_sort_x:
		valid = False
		for neutron in total_list_sort_x:
			if history != proton['x']:
				if proton['y'] == neutron['y']:
					if proton['x'] < neutron ['x']:
						if neutron['x'] <= proton['x'] + proton['w'] + weight:
							sub_list.append(proton['d']+neutron['d'])
							valid=True
							history=neutron['x']
						else:
							sorted_flag=True
					else:
						sorted_flag=True
				else:
					sorted_flag=True
			else:
				sorted_flag=False

		if sorted_flag==True and valid==False:
			sub_list.append(proton['d'])

	count = 0
	for string in sub_list:
		if count == 0:
			width = int(string)
		else:
			length = length + int(string)
		count = count + 1
		dimensions = dimensions + "\n" + string

	print "Length: ",length,"\n Width: ",width
	print "\n",dimensions

	while not rospy.is_shutdown():
		pub.publish(dimensions)
		rate.sleep()
		if cv2.waitKey(1) == 27:
			break

if __name__ == "__main__":

	digit_list = []

	region = 0

	capture = cv2.VideoCapture(0)

	samples = np.loadtxt(FOLDER+'generalsamples.data',np.float32)
	responses = np.loadtxt(FOLDER+'generalresponses.data',np.float32)
	responses = responses.reshape((responses.size,1))

	model = cv2.ml.KNearest_create()
	model.train(samples,cv2.ml.ROW_SAMPLE,responses)

	while True:
		# read each frame
		ret, image = capture.read()

		image_analysis = image

		image_gray = cv2.cvtColor(image_analysis, cv2.COLOR_BGR2GRAY)
		ret, thresholded_image = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY_INV)
		_, contours, hierarchy = cv2.findContours(thresholded_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		for contour in contours:
			if cv2.contourArea(contour)<2000 and cv2.contourArea(contour)>500:
				area = cv2.contourArea(contour)
				[x,y,w,h] = cv2.boundingRect(contour)
				if h>35:
					crop_img = image_analysis[y:y+h, x:x+w]
					cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),1)
					region = thresholded_image[y:y+h,x:x+w]
	     			region_crop = cv2.resize(region,(10,10))
	       			region_crop = region_crop.reshape((1,100))
	       			region_crop = np.float32(region_crop)
	       			retval, results, neigh_resp, dists = model.findNearest(region_crop, k = 2)
	       			string = str(int((results[0][0])))

	       			cv2.putText(image,string,(x,y-15),2,1,(0,0,255))
	    			digit_list.append(Digits(x,y,w,h,string))

	    	if cv2.waitKey(1) == 13:
	    		dispatch_dimensions(digit_list)

	    	else:
	    		digit_list[:] = []

		cv2.imshow('Beta test window',image)

		if cv2.waitKey(1) == 27:
			break

	capture.release()
	cv2.destroyAllWindows()