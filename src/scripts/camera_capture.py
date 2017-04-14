#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import operator
import rospy
import sys
from cosrap.msg import dimensions as dim
from os.path import expanduser

HOME = expanduser('~')
FOLDER = HOME + '/catkin_ws/src/cosrap/src/scripts/'

class Digits:

	def __init__(self,x,y,h,w,s):
		self.x_column = x
		self.y_row = y
		self.height = h
		self.width = w
		self.digit = s

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
		digit_dict = {
			'x': digits.x_column,
			'y': digits.y_row,
			'w': digits.width,
			'h': digits.height,
			'd': digits.digit,
			}
		total_list.append(digit_dict)

	total_list_sort = total_list
	total_list_sort.sort(key=operator.itemgetter('y', 'x', 'w'))

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

	total_list_sort_x = total_list_sort
	total_list_sort_x.sort(key=operator.itemgetter('x'))

	# string stitching

	for proton in total_list_sort_x:
		valid = False
		for neutron in total_list_sort_x:
			if history != proton['x']:
				if proton['y'] == neutron['y']:
					if proton['x'] < neutron['x']:
						if neutron['x'] <= proton['x'] + proton['w'] + weight:
							sub_list.append(proton['d'] + neutron['d'])
							valid = True
							history = neutron['x']
						else:
							sorted_flag = True
					else:
						sorted_flag = True
				else:
					sorted_flag = True
			else:
				sorted_flag = False

		if sorted_flag == True and valid == False:
			sub_list.append(proton['d'])

	count = 0
	for string in sub_list:
		if count == 0:
			width = int(string)
		else:
			length = length + int(string)
		count = count + 1
		dimensions = dimensions + '\n' + string

	print 'Length: ', length, '\n Width: ', width
	print '\n', dimensions

	while not rospy.is_shutdown():
		pub.publish(dimensions)
		rate.sleep()
		if cv2.waitKey(1) == 27:
			sys.exit()


if __name__ == '__main__':

	digit_list = []

	region = 0

	capture = cv2.VideoCapture(0)
	width_grid = capture.get(3)
	height_grid = capture.get(4)

	samples = np.loadtxt(FOLDER + 'generalsamples.data', np.float32)
	responses = np.loadtxt(FOLDER + 'generalresponses.data', np.float32)
	responses = responses.reshape((responses.size, 1))

	model = cv2.ml.KNearest_create()
	model.train(samples, cv2.ml.ROW_SAMPLE, responses)

	while True:

		# read each frame

		ret, image = capture.read()

		image_analysis = image

		image_gray = cv2.cvtColor(image_analysis, cv2.COLOR_BGR2GRAY)
		ret, thresholded_image = cv2.threshold(image_gray, 127, 255,cv2.THRESH_BINARY_INV)
		_, contours, hierarchy = cv2.findContours(thresholded_image,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		for contour in contours:
			if cv2.contourArea(contour) < 2000 and cv2.contourArea(contour) > 500:
				area = cv2.contourArea(contour)
				[x, y, w, h] = cv2.boundingRect(contour)
				if h > 35 and h < 60:
					crop_img = image_analysis[y:y + h, x:x + w]
					cv2.rectangle(image, (x, y), (x + w, y + h), (255,0, 0), 1)
					region = thresholded_image[y:y + h, x:x + w]
				region_crop = cv2.resize(region, (10, 10))
				region_crop = region_crop.reshape((1, 100))
				region_crop = np.float32(region_crop)
				(retval, results, neigh_resp, dists) = model.findNearest(region_crop, k=2)
				string = str(int(results[0][0]))
				cv2.putText(image,string,(x, y - 15),2,1,(0, 0, 255))
				digit_list.append(Digits(x, y, w, h, string))

		if cv2.waitKey(1) == ord('c'):
			dispatch_dimensions(digit_list)
		else:

			digit_list[:] = []

		# read each frame

		cv2.rectangle(image, (0, 0), (int(width_grid / 3),int(height_grid)), (255, 255, 0), 1)
		cv2.rectangle(image, (int(2 * width_grid / 3), 0),(int(width_grid / 3), int(height_grid)), (255,255,0), 1)
		cv2.rectangle(image, (int(2 * width_grid / 3) + int(width_grid/ 3), 0), (int(width_grid / 3),int(height_grid)), (255, 255, 0), 1)
		cv2.rectangle(image, (0, 0), (int(width_grid), int(height_grid/ 3)), (255, 255, 0), 1)
		cv2.rectangle(image, (0, int(2 * height_grid / 3)),(int(width_grid), int(height_grid / 3)), (255,255, 0), 1)
		cv2.rectangle(image, (0, int(2 * height_grid / 3) + int(height_grid / 3)), (int(width_grid),int(height_grid / 3)), (255, 255, 0), 1)
		cv2.imshow('User input Window', image)

		if cv2.waitKey(1) == 27:
			sys.exit()

	capture.release()
	cv2.destroyAllWindows()
