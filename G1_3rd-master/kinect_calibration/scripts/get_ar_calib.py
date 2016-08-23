#!/usr/bin/env python

import rospy
import tf
import numpy
import yaml
from math import pi

from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs.msg

# TODO: add averaging/filtering, auto-add to param server or urdf or file

config_folder = rospy.get_param('object_tracker/config_folder')
with open(config_folder+'ar_calib.yaml', 'r') as f:
    params = yaml.load(f)

class markerSubscriber():
    def __init__(self, markernum=2):
        self.sub = rospy.Subscriber("/visualization_marker", Marker, self.callback)
        self.markernum = markernum
        self.pose = None
    def callback(self, data):
        self.pose = data.pose

def getPoseFromMatrix(matrix):
    trans, quat = getTfFromMatrix(numpy.linalg.inv(matrix))
    return Pose(position=Point(*trans), orientation=Quaternion(*quat))

def getMatrixFromPose(pose):
    trans = (pose.position.x, pose.position.y, pose.position.z )
    rot = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    return tf.transformations.compose_matrix(translate = trans, angles = rot)

def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles)

def lookupTransform(tf_listener, target, source):
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(target, source, rospy.Time(0), rospy.Duration(4.0))
            break
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            continue
    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time(0))
    euler = tf.transformations.euler_from_quaternion(rot)
    source_target = tf.transformations.compose_matrix(translate = trans,
                                                     angles = euler)
    #print "looked up transform from", source, "to", target, "-", source_target
    return source_target

def create_marker(ns, id_num, shape_type, pose, color, scale):
    # Create rviz marker message
    marker = Marker()
    marker.header.frame_id = "/base"
    marker.ns = ns
    marker.id = id_num
    marker.type = shape_type
    marker.action = Marker.ADD
    marker.pose = pose
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    return marker

markernum = params['markernum']
measured_translation = params['measured_translation']
measured_rot = numpy.array(params['measured_rot']).reshape((3,3))
frame = params['frame']
squaredims = tuple(params['squaredims'])

rospy.init_node("get_ar_calib")

tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()

marker_pub = rospy.Publisher("ar_calib_markers", MarkerArray)

rate = rospy.Rate(100)

reference_marker_trans = numpy.dot(numpy.linalg.inv(measured_rot),
                               -numpy.array(measured_translation).reshape((3, 1))).flatten()
print reference_marker_trans
# Calculate transform from forearm to marker
reference_marker = tf.transformations.compose_matrix(
                translate = reference_marker_trans,
                angles = tf.transformations.euler_from_matrix(measured_rot))
print reference_marker

marker_sub = markerSubscriber()

# Publish transform and marker
while not rospy.is_shutdown():

    # base to forearm
    print frame
    base_reference = lookupTransform(tf_listener, frame, '/base')

    # Compose transforms
    # base to marker = forearm to marker * base to forearm
    base_marker = reference_marker.dot(base_reference)
    trans, rot = getTfFromMatrix(numpy.linalg.inv(base_marker))
    tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "/ar_marker_1", "/base")
    marker_pose = getPoseFromMatrix(base_marker)

    # marker to camera
    marker_camera = lookupTransform(tf_listener, '/camera_link', '/ar_marker_1')
    #marker_camera = lookupTransform(tf_listener,  '/ar_marker_1', '/camera_rgb_optical_frame')


    # base to camera = marker to camera * base to marker
    base_camera = marker_camera.dot(base_marker)

    # base to camera link = camera_rgb to camera link * base to camera_rgb
    # rgb_link = lookupTransform(tf_listener, '/camera_link', '/camera_rgb_optical_frame')
    # base_camera_rgb = rgb_link.dot(base_camera_rgb)
    trans, rot = getTfFromMatrix(numpy.linalg.inv(base_camera))

    # ref to camera = base to camera * ref to base
    ref_frame = '/right_hand'
    ref_base = lookupTransform(tf_listener, '/base', ref_frame)
    ref_camera = base_camera.dot(ref_base)
    trans_ref, rot_ref = getTfFromMatrix(numpy.linalg.inv(ref_camera))
    # write to yaml based on this trans and rot from ref_frame to camera_frame


    tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "/camera_link", "/base")
    camera_pose = getPoseFromMatrix(base_camera)
    
    marker_msg = create_marker("marker_pose", 44, Marker.CUBE, marker_pose, (0, 255, 0), squaredims )

    camera_msg = create_marker("camera_pose", 1337, Marker.ARROW, camera_pose, (0, 0, 255), (0.2, 0.01, 0.01))

    # Now refine the transform using least squares
    # Get the pose of the marker from ar_pose_marker in the camera frame
    msg = [marker_msg, camera_msg]

    marker_pub.publish(msg)
    print "camera transfer is: " + str(trans)
    rate.sleep()

print "Writing transform to yaml file"
# Write to yaml file
f = open(config_folder+"/base_camera_tf.yaml", 'w')
lines = ['trans: [', 'rot: [']
for elem in trans:
    lines[0] += str(elem) + ', '
lines[0] += ']\n'
for elem in rot:
    lines[1] += str(elem) + ', '
lines[1] += ']\n'
lines.append('parent: /base\n')
lines.append('child: /camera_link\n')
f.writelines(lines)
f.close()
