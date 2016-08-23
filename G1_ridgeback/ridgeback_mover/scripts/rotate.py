#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from ridgeback_mover.srv import RotateService

mover = None
listener = None
robot_yaw = 0.0
object_yaw = 0.0

def send_move_cmd(cmd):
	global mover

	to_send = Twist()
	if cmd == 'START_L':
		to_send.linear.x = 0.0
		to_send.linear.y = 0.0
		to_send.linear.z = 0.0
		to_send.angular.x = 0.0
		to_send.angular.y = 0.0
		to_send.angular.z = 0.05
	if cmd == 'START_R':
		to_send.linear.x = 0.0
		to_send.linear.y = 0.0
		to_send.linear.z = 0.0
		to_send.angular.x = 0.0
		to_send.angular.y = 0.0
		to_send.angular.z = -0.05
	elif cmd == 'STOP':
		to_send.linear.x = 0.0
		to_send.linear.y = 0.0
		to_send.linear.z = 0.0
		to_send.angular.x = 0.0
		to_send.angular.y = 0.0
		to_send.angular.z = 0.0
	rate = rospy.Rate(10)
	mover.publish(to_send)
	rate.sleep()

def get_robot_yaw(pose_data):
	global robot_yaw
	quaternion = (
	    pose_data.orientation.x,
	    pose_data.orientation.y,
	    pose_data.orientation.z,
	    pose_data.orientation.w)
	robot_euler = tf.transformations.euler_from_quaternion(quaternion)
	robot_yaw = robot_euler[2]

def get_object_yaw(look_at):
	global object_yaw
	global listener

	listener.waitForTransform("anchor", look_at, rospy.Time(0), rospy.Duration(4.0))
	(look_at_pos, look_at_rot) = listener.lookupTransform("anchor", look_at, rospy.Time(0))
	look_at_euler = tf.transformations.euler_from_quaternion(look_at_rot)
	look_at_yaw = look_at_euler[2]


def handle_rotate_service(req):
	global mover
	global object_yaw, robot_yaw

	mover = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	getter = rospy.Subscriber('/robot_pose', Pose, get_robot_yaw)
	rospy.wait_for_message('/robot_pose',Pose)
	get_object_yaw(req.world_object)

	while (math.fabs(to_turn) > 0.034):
		to_turn = np.deg2rad(object_yaw-robot_yaw)
		print (("TURN %f DEGREES")%(np.rad2deg(to_turn)))
		if to_turn < 0:
			send_move_cmd("START_R")
		elif to_turn > 0:
			send_move_cmd("START_L")
		else:
			send_move_cmd("STOP")
	send_move_cmd("STOP")
	return True
		
def rotate_service_server():
	global listener

	rospy.init_node('rotate_ridgeback')
	listener = tf.TransformListener()
	s = rospy.Service('rotate_ridgeback', RotateService, handle_rotate_service)
	print "Ready to rotate Ridgeback"
	rospy.spin()

if __name__ == '__main__':
	rotate_service_server()