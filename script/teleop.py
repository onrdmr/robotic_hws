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


### Function to identify and locate obstacles around the robot
def laserCallBack(data):
	ranges = np.array(data.ranges)
	global Proximity
	global obsLocate
	Proximity = min(ranges)
	obsLocate = np.where(ranges == Proximity)

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

def constrain(input, low, high):
	if input < low:
	  input = low
	elif input > high:
	  input = high
	else:
	  input = input

	return input

def checkLinearLimitVelocity(vel):
	vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
	return vel

def checkAngularLimitVelocity(vel):
	vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
	return vel


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

	rospy.init_node('teleop_node')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber("scan", LaserScan,laserCallBack)
	turtlebot3_model = rospy.get_param("model", "burger")

	status = 0
	target_linear_vel   = 0.0
	target_angular_vel  = 0.0
	control_linear_vel  = 0.0
	control_angular_vel = 0.0

	try:
		print(msg)
		while(1):
			key = getKey()
			if key == 'w' :
				target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
				status = status + 1
				print()
				print(vels(target_linear_vel,target_angular_vel))
				print()				
				print("The distance to the closest obstacle is ",Proximity)
				locateObstacle(obsLocate)
				print()

			elif key == 'x' :
				target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
				status = status + 1
				print()
				print(vels(target_linear_vel,target_angular_vel))
				print()
				print("The distance to the closest obstacle is ",Proximity)
				locateObstacle(obsLocate)
				print()

			elif key == 'd' :
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_vel,target_angular_vel))
				print()
				print("The distance to the closest obstacle is ",Proximity)
				print()
				locateObstacle(obsLocate)
				print()

			elif key == 'a' :
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
				status = status + 1
				print()
				print(vels(target_linear_vel,target_angular_vel))				
				print()
				print("The distance to the closest obstacle is ",Proximity)
				print()
				locateObstacle(obsLocate)
				print()

			elif key == ' ' or key == 's' :
				target_linear_vel   = 0.0
				control_linear_vel  = 0.0
				target_angular_vel  = 0.0
				control_angular_vel = 0.0
				print()
				print(vels(target_linear_vel, target_angular_vel))
				print()
				print("The distance to the closest obstacle is ",Proximity)
				print()
				locateObstacle(obsLocate)
				print()

			else:
				if (key == '\x03'):
					break

			twist = Twist()

			control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
			twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

			control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

			pub.publish(twist)

	except:
		print("Node not parsed with the bot, failed to communicate with the environment")