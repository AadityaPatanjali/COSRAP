import rospy
from geometry_msgs.msg import Twist
from cosrap.msg import dimensions as dim
from math import pi as PI
import Robot


class Command()

	vel_msg = Twist()
	pub = None
	dimensions = list()
	len_bred = list()
	time = list()
	def __init__(self):
		rospy.Subscriber("dim_math", dim, self.callback)
		rospy.wait_for_message("dim_math",dim)
		self.dimensions = self.init_dim()
		self.init_time()

	def main(self):
		# Set the trim offset for each motor (left and right).  This is a value that
		# will offset the speed of movement of each motor in order to make them both
		# move at the same desired speed.  Because there's no feedback the robot doesn't
		# know how fast each motor is spinning and the robot can pull to a side if one
		# motor spins faster than the other motor.  To determine the trim values move the
		# robot forward slowly (around 100 speed) and watch if it veers to the left or
		# right.  If it veers left then the _right_ motor is spinning faster so try
		# setting RIGHT_TRIM to a small negative value, like -5, to slow down the right
		# motor.  Likewise if it veers right then adjust the _left_ motor trim to a small
		# negative value.  Increase or decrease the trim value until the bot moves
		# straight forward/backward.
		LEFT_TRIM   = 0
		RIGHT_TRIM  = 0


		# Create an instance of the robot with the specified trim values.
		# Not shown are other optional parameters:
		#  - addr: The I2C address of the motor HAT, default is 0x60.
		#  - left_id: The ID of the left motor, default is 1.
		#  - right_id: The ID of the right motor, default is 2.
		robot = Robot.Robot(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)
		teleport(robot)
		# Now move the robot around!
		# Each call below takes two parameters:
		#  - speed: The speed of the movement, a value from 0-255.  The higher the value
		#           the faster the movement.  You need to start with a value around 100
		#           to get enough torque to move the robot.
		#  - time (seconds):  Amount of time to perform the movement.  After moving for
		#                     this amount of seconds the robot will stop.  This parameter
		#                     is optional and if not specified the robot will start moving
		#                     forever.
		for ele in self.time:
			if ele[1] == 0:
				robot.forward(rpm,ele[0])
			elif ele[1]>0:
				robot.left(rpm,ele[1])
			else:
				robot.right(rpm,ele[1])
		rospy.signal_shutdown('')
		rospy.spin()


		# That's it!  Note that on exit the robot will automatically stop moving.
	def init_dim(self):
		# print field dimensions from image processing node
		print self.len_bred
		dimensions = list()
		dimensions.append([self.len_bred[1],0,1])
		dimensions.append([0,-90,0])
		dimensions.append([self.len_bred[0],0,1])
		dimensions.append([0,-90,0])
		dimensions.append([self.len_bred[3],0,1])
		dimensions.append([0,-90,0])
		dimensions.append([self.len_bred[0],0,1])
		dimensions.append([0,-90,0])
		dimensions.append([self.len_bred[1],0,0])
		dimensions.append([self.len_bred[2],0,1])
		return dimensions

	def init_time(
		self,
		wheel_radius = 3.0, #cm
		base_length = 9.8, #cm
		rpm = 150.0
		):

		distance_per_revolution = 2*pi*wheel_radius
		velocity = distance_per_revolution*rpm/60 #cm/s
		angular_velocity = velocity*360/(2*pi*base_length) #degrees/s
		self.time = [(dim/velocity,ang/angular_velocity,marker) for dim,ang,marker in self.dimensions]

	def callback(self,data):
		dimen = data.dim
		self.len_bred = [int(item) for item in dimen.split()]
		print self.len_bred

	def teleport(robot):
		# Write code to move the robot to a specific location
		# robot.forward(,)
		# if condition:
		# 	robot.left(,)
		# else:
		# 	robot.right(,)

if __name__ =='__main__':
	try: 
#		rospy.loginfo('executing main')
		comm=Command()
#		rospy.loginfo('initialized Command')
		comm.main()
	except rospy.ROSInterruptException:
 
#		rospy.loginfo('not main')
		pass