#!/usr/bin/env python

import rospy
import tf
import math
import numpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from ridgeback_mover.srv import TranslateService

# DECLARE GLOBAL VARS #
x_pt = 6.0
y_pt = 6.0
z_pt = 6.0
x_err = 6.0
y_err = 6.0
mover = None
br = None
br2 = None
listener = None
threshold = 0.005

# MOVE COMMAND SENDER #
def send_move_cmd(cmd, offset_x, offset_y):
	global mover

	# INIT Twist MESSAGE #
	to_send = Twist()

	if cmd == 'START':
		# COMPUTE VELOCITY #
		dist = math.sqrt(math.pow(offset_x,2)+math.pow(offset_y,2))
		x_vel = 0.05 * (offset_x/dist)
		y_vel = 0.05 * (offset_y/dist)
		# SET VELOCITY #
		to_send.linear.x = x_vel
		to_send.linear.y = y_vel
		to_send.linear.z = 0.0
		to_send.angular.x = 0.0
		to_send.angular.y = 0.0
		to_send.angular.z = 0.0
	elif cmd == 'STOP':
		to_send.linear.x = 0.0
		to_send.linear.y = 0.0
		to_send.linear.z = 0.0
		to_send.angular.x = 0.0
		to_send.angular.y = 0.0
		to_send.angular.z = 0.0
	rate = rospy.Rate(10)
	# SEND VELOCITY #
	mover.publish(to_send)
	rate.sleep()
	return

# GET TARGET POINT #
def get_point(point_click):
	global x_pt, y_pt, z_pt

	point_data = (point_click.x, point_click.y, point_click.z)
	x_pt = point_data[0]
	y_pt = point_data[1]
	z_pt = point_data[2]
	return
	
# GET CURRENT POSITION #
def get_curr_xy(pose_data):
	global br

	pose_position = (pose_data.position.x, pose_data.position.y, pose_data.position.z)
	pose_orientation = (pose_data.orientation.x, pose_data.orientation.y, pose_data.orientation.z, pose_data.orientation.w)
	x = pose_position[0]
	y = pose_position[1]
	z = pose_position[2]
	xo = pose_orientation[0]
	yo = pose_orientation[1]
	zo = pose_orientation[2]
	wo = pose_orientation[3]

	# SET TF FROM POSITION #
	br.sendTransform((x, y, z), (xo, yo, zo, wo), rospy.Time.now(), "baxter", "map")
	return

def handle_translate_service(req):
	global mover
	global threshold
	global br2
	global x_err, y_err
	global x_pt, y_pt, z_pt

	# SET TWIST PUBLISHER #
	mover = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	# SET POSE SUBSCRIBER #
	getter = rospy.Subscriber('/robot_pose', Pose, get_curr_xy)
	rospy.wait_for_message('/robot_pose', Pose)
	# SET TARGET SUBSCRIBER #
	clicked = rospy.Subscriber('/pseudo_click', Point, get_point)
	rospy.wait_for_message('/pseudo_click', Point)

	# MOVE TILL THRESHOLD REACHED #
	while ((math.fabs(x_err) > threshold) or (math.fabs(y_err) > threshold)):
		# SET TF FOR TARGET #
		br2.sendTransform((x_pt, y_pt, z_pt), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "point", "map")

		# GET TF FOR BAXTER --> TARGET #
		listener.waitForTransform("baxter", "point", rospy.Time(0), rospy.Duration(4.0))
		(go_to_pos, go_to_rot) = listener.lookupTransform("baxter", "point", rospy.Time(0))

		# UPDATE OFFSETS #
		x_err, y_err = go_to_pos[0], go_to_pos[1]
		# ACTUALLY MOVE #
		send_move_cmd("START", x_err, y_err)
	
	# RIJBEK STAAHP. JUST STAAHP #
	send_move_cmd("STOP", 0, 0)
	return True

def translate_service_server():
	global br
	global br2
	global listener

	rospy.init_node('translate_ridgeback')
	br = tf.TransformBroadcaster()
	br2 = tf.TransformBroadcaster()
	listener = tf.TransformListener()
	s = rospy.Service('translate_ridgeback', TranslateService, handle_translate_service)
	print "Ready to translate Ridgeback"
	rospy.spin()

if __name__ == '__main__':
	translate_service_server()