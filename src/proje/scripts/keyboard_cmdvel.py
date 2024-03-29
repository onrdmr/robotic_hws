#!/usr/bin/env python
import roslib; roslib.load_manifest('proje')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
# print("pyversion" , sys.version)


## this two for camera callback
from camera_callback import camera_callback, keyboard_callback, trajectory_callback
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker

import threading

from qr_test import qr_recognition

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		# '<':(-1,0,0,1),
		'>':(-1,-1,0,0),
		# '>':(1,0,0,-1),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
	rospy.init_node('keyboard_cmdvel')
	settings = termios.tcgetattr(sys.stdin)

	modified_camera_pub = rospy.Publisher('/rtg/camera/rgb/modified_image', Image, queue_size = 1)
	object_trace_publisher = rospy.Publisher('/object_trace_publisher', Marker, queue_size = 1)

	rospy.Subscriber('/rtg/camera/rgb/image_raw', Image, camera_callback, (modified_camera_pub))
	pub = rospy.Publisher('/rtg/cmd_vel', Twist, queue_size = 1)
	pub_key = rospy.Publisher('/rtg/key', String, queue_size = 10)
	rospy.Subscriber('/rtg/key', String, keyboard_callback)

	rospy.Subscriber('//rtg/odom', Odometry, trajectory_callback, (object_trace_publisher))

	## QR thread listens lock_qr
	my_thread = threading.Thread(target=qr_recognition, args=("/home/onur/robotic_hws/src/proje/scripts/rgb_location.png",))
	my_thread.daemon = True
	my_thread.start()


	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			pub_key.publish(key)
				# this 
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(twist)

	except:
		# lock = open("file_qr.lock", 'w')
		# lock.write("-1")
		print("error")

	finally:
		# lock = open("file_qr.lock", 'w')
		# lock.write("-1")
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


