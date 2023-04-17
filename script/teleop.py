#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
from sensor_msgs.msg import LaserScan
import tty, termios
import numpy as np

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

global Proximity 
global obsLocate
global object_loc
object_loc = None
Proximity = None
obsLocate = None

init_trans_x=0.011008159263360684
init_trans_y=-5.156497392615714
init_trans_z=0.011008159263360684

init_rot_x=4.553158104623501e-06
init_rot_y=8.128222477999093e-06
init_rot_z=-3.3588706412369306e-06


msg = """
Manual Keyboard Control For robot
---------------------------
Moving around using the following keys:
w- Forward
s- Stop
a- Turn left
d- Turn right
        w
   a    s    d
        x
CTRL-C to quit
"""


class Movement :
	def __init__(self):
		rospy.init_node('move_robot_node', anonymous=False)
		self.pub_move = rospy.Publisher("/rtg/cmd_vel", Twist, queue_size=10)
		self.move = Twist()
	def publish_vel(self):
		self.pub_move.publish(self.move)
	def move_forward(self):
		self.move.linear.x=100
		self.move.angular.z=0.0
	def angular_right(self):
		self.move.angular.x = 10
		# self.move.angular.z = 1
	def move_backward(self):      
		self.move.linear.x=-1
		self.move.angular.z=0.0

	def stop(self):        
		self.move.linear.x=0
		self.move.angular.z=0.0  

### Function to identify and locate obstacles around the robot
def laserCallBack(data):
	twist = Twist()
	ranges = np.array(data.ranges)
	global Proximity
	global obsLocate
	Proximity = min(ranges)
	obsLocate = np.where(ranges == Proximity)

	twist.linear.x = 1


def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vels(target_linear_vel, target_angular_vel):
	return "The current velocities of the Robot are:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output



### Function to identify exact location of obstacles relative to the robot
def locateObstacle(ran):
	if ran[0] >=0 and ran[0] <=250:
		object_loc = 1
		print("Obstacle is detected to the RIGHT")

	elif ran[0] > 250  and ran[0] <= 500:
		object_loc = 2
		print("Obstacle is detected in FRONT")

	elif ran[0] > 500 and ran[0] <=720:
		object_loc = 3
		print("Obstacle is detected to the LEFT")

	else:
		object_loc = 4
		print("Path clear!")

	return object_loc



if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)

	# rospy.init_node('teleop_node')
	# pub = rospy.Publisher('/rtg/cmd_vel', Twist, queue_size=10)
	# sub = rospy.Subscriber("/rtg/hokuyo", LaserScan, laserCallBack)
	# # turtlebot3_model = rospy.get_param("model", "burger")
	

	# status = 0
	# target_linear_vel   = 0.0
	# target_angular_vel  = 0.0
	# control_linear_vel  = 0.0
	# control_angular_vel = 0.0

	mov = Movement()
	rate = rospy.Rate(1)

	while not rospy.is_shutdown() :

		# movement = input('Enter desired movement: ')
		movement = None
		if movement == 'forward':
			mov.move_forward()

		if movement == 'backward':
			mov.move_backward()

		if movement == 'right':
			mov.angular_right()

		if movement == 'stop':
			mov.stop()
		mov.publish_vel()
		rate.sleep()

	# Todos : 

	# conditions to debug
	# hızlanmayı yavaştan hızlı yapamak gerekli

	# 1: take snapshot of laser scan
	# 2: mapping for acceleration (set next yaw and velocity) -- this will be extended
	# 3: reset robot position yaw pitch roll to initial position

	# try:
	# 	print(msg)
	# 	twist = Twist()
	# 	while(1):
	# 		key = getKey()
	# 		if key == 'w' :
	# 			# target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
	# 			# status = status + 1
	# 			print()
	# 			print(vels(1.1, 1.0))
	# 			print()				
	# 			# print("The distance to the closest obstacle is ",Proximity)
	# 			# locateObstacle(obsLocate)
	# 			twist.linear.x = 1.1
	# 			twist.cmd_vel.linear.y = 1.0
	# 			# twist.cmd_vel.linear.z = 0.0;
	# 			twist.cmd_vel.angular.x = 0.0
	# 			twist.cmd_vel.angular.y = 0.0
				
	# 			print()
	# 		if key == 's':
	# 			twist.cmd_vel.linear.x = 0.0
	# 			twist.cmd_vel.linear.y = 0.0
	# 			# twist.cmd_vel.linear.z = 0.0;
	# 			twist.cmd_vel.angular.x = 0.0
	# 			twist.cmd_vel.angular.y = 0.0
				
	# 		else:
	# 			if (key == '\x03'):
	# 				break


	# 		control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
	# 		twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

	# 		control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
	# 		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

	# 		pub.publish(twist)

	# except:
	# 	print("Node not parsed with the bot, failed to communicate with the environment")