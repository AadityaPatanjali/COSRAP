#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import pi as PI


class Command:
	vel_msg = Twist()
	pub = None
	dimensions = list()
	def __init__(self):
		self.dimensions = self.init_dim()
		self.vel_msg.linear.y = 0
		self.vel_msg.linear.z = 0
		self.vel_msg.angular.x = 0
		self.vel_msg.angular.y = 0
		rospy.init_node('vel_ang')
		self.pub = rospy.Publisher('turtle1/cmd_vel',Twist, queue_size = 10)
#		rospy.loginfo('initialized')
	def main(self):

#		rospy.loginfo('dimensions initialized')
		for ele in self.dimensions:
			if(ele[1]==0): self.move(ele[0])
			else: self.rotate(ele[0],ele[1])
#		rospy.loginfo('finishing motion')
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.pub.publish(self.vel_msg)
		rospy.signal_shutdown('')
		rospy.spin()


	def move(self,distance):
		speed = 1
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
#		if (distance<0): self.vel_msg.linear.x = -speed
#		else: self.vel_msg.linear.x = speed
		self.vel_msg.linear.x = speed
		self.vel_msg.angular.z = 0
#		rospy.loginfo('in move while %.2f',self.vel_msg.linear.x)

		while(current_distance<distance):
			t1 = rospy.Time.now().to_sec()
			current_distance = speed*(t1-t0)
			self.pub.publish(self.vel_msg)

	def rotate(self,distance,angle):
		speed = 45
		angular_speed =speed*2*PI/360
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
		relative_angle =abs(angle)*2*PI/360
#		rospy.loginfo('in rotate while %.2f,%.2f.',distance,angle)
		if(angle<0): self.vel_msg.angular.z = -angular_speed 
		else: self.vel_msg.angular.z = angular_speed
		self.vel_msg.linear.x = distance
		while(current_angle < relative_angle):
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed*(t1-t0)
			self.pub.publish(self.vel_msg)
	def init_dim(self):
		# input field dimensions from image processing node
		len_bred = [30,60,15,40,15]
		len_bred = [dim*10/60 for dim in len_bred]

		dimensions = list()
		dimensions.append([len_bred[2],0,1])
		dimensions.append([0,-90,0])
		dimensions.append([len_bred[0],0,1])
		dimensions.append([0,-90,0])
		dimensions.append([len_bred[2],0,1])
		dimensions.append([0,-90,0])
		dimensions.append([len_bred[0],0,1])
		dimensions.append([0,-90,0])
		dimensions.append([len_bred[2],0,0])
		dimensions.append([len_bred[3]-len_bred[2],0,1])

		return dimensions

if __name__ =='__main__':
	try: 
#		rospy.loginfo('executing main')
		comm=Command()
#		rospy.loginfo('initialized Command')
		comm.main()
	except rospy.ROSInterruptException:
 
#		rospy.loginfo('not main')
		pass
		
