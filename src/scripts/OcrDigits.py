#!/usr/bin/env python

import numpy as np

class Digits:
	def __init__(self,x,y,h,w,s):
		self.x_column = x
		self.y_row = y
		self.height = h
		self.width = w
		self.digit = s