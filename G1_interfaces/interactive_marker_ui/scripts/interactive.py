#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from world_model_msgs.srv import QueryLocations
from world_model_msgs.msg import Location
from geometry_msgs.msg import Point
from interactive_marker_ui.srv import *

server = None
READ = 0
UPDATE = 1
DELETE = 2
upd_status = "Not editing anything"
upd_name = None
upd_x, upd_y, upd_z = None, None, None

def handle_location_update_status(req):
	return location_update_statusResponse(upd_status)

def location_update_status_server():
	s1 = rospy.Service('location_update_status', location_update_status, handle_location_update_status)
	print "Now serving location update status"

def handle_location_update_confirm(req):
	write_stat = write_wm_locations(upd_name, upd_x, upd_y, upd_z)
	return location_update_confirmResponse(write_stat)

def location_update_confirm_server():
	s2 = rospy.Service('location_update_confirm', location_update_confirm, handle_location_update_confirm)
	print "Now serving location update confirm"

def processFeedback(feedback):
	global upd_status, upd_name, upd_x, upd_y, upd_z

	if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
		upd_status =  "Now editing %s"%(feedback.marker_name)
		upd_name = feedback.marker_name
	if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
		p = feedback.pose.position
		upd_status = ("Location %s is now at (%.2f,%.2f,%.2f)")%(feedback.marker_name, p.x, p.y, p.z)
		upd_x = p.x
		upd_y = p.y
		upd_z = p.z

def makeBoxControl( msg ):
	marker = Marker()
	marker.type = Marker.CUBE
	marker.scale.x = 0.2
	marker.scale.y = 0.2
	marker.scale.z = 0.01
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	text_marker = Marker()
	text_marker.type = Marker.TEXT_VIEW_FACING
	text_marker.pose.position.z = 0.2
	text_marker.scale.x = 0.2
	text_marker.scale.y = 0.2
	text_marker.scale.z = 0.01
	text_marker.color.r = 1.0
	text_marker.color.g = 1.0
	text_marker.color.b = 1.0
	text_marker.color.a = 1.0
	text_marker.text = msg.name

	box_control = InteractiveMarkerControl()
	box_control.always_visible = True
	box_control.markers.append( marker )
	box_control.markers.append( text_marker )
	msg.controls.append( box_control )

	control = InteractiveMarkerControl()
	control.name = "move_x"
	control.description = "Movement Along x-axis"
	control.always_visible = True
	control.orientation.w = 1.0
	control.orientation.x = 1.0
	control.orientation.y = 0.0
	control.orientation.z = 0.0
	control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
	msg.controls.append( control )
	
	control2 = InteractiveMarkerControl()
	control2.name = "move_y"
	control.description = "Movement Along y-axis"
	control2.always_visible = True
	control2.orientation.w = 1.0
	control2.orientation.x = 0.0
	control2.orientation.y = 0.0
	control2.orientation.z = 1.0
	control2.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
	msg.controls.append( control2 )

	return

def setMarker(set_marker_name, set_marker_desc, set_pos_x, set_pos_y, set_pos_z):
	global server

	# create an interactive marker for our server
	int_marker = InteractiveMarker()
	int_marker.header.frame_id = "base"
	int_marker.name = set_marker_name
	int_marker.description = set_marker_desc

	int_marker.pose.position.x = set_pos_x
	int_marker.pose.position.y = set_pos_y
	int_marker.pose.position.z = set_pos_z

	makeBoxControl(int_marker)
	
	server.insert(int_marker, processFeedback)

def read_wm_locations():
	global READ, UPDATE, DELETE
	rospy.wait_for_service('/world_model/query_locations')
	try:
		location_reader = rospy.ServiceProxy('/world_model/query_locations', QueryLocations)
		blank_loc = Location()
		blank_loc.id = ""
		blank_pt = Point()
		blank_pt.x = 0.0
		blank_pt.y = 0.0
		blank_pt.z = 0.0
		blank_loc.position = blank_pt
		location_list = location_reader([blank_loc], '', READ)
		return location_list.locations
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def write_wm_locations(loc_id,write_x,write_y,write_z):
	global READ, UPDATE, DELETE
	rospy.wait_for_service('/world_model/query_locations')
	try:
		location_writer = rospy.ServiceProxy('/world_model/query_locations', QueryLocations)
		set_loc = Location()
		set_loc.id = loc_id
		set_pt = Point()
		set_pt.x = write_x
		set_pt.y = write_y
		set_pt.z = write_z
		set_loc.position = set_pt
		location_list = location_writer([set_loc], loc_id, UPDATE)
		location_parser(read_wm_locations())
		return location_list.success
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def location_parser(get_locations):
	global server
	print len(get_locations)
	for i in range(0, len(get_locations)):
		set_name = get_locations[i].id
		set_desc = set_name
		set_x = get_locations[i].position.x
		set_y = get_locations[i].position.y
		set_z = get_locations[i].position.z
		setMarker(set_name,'marker for ' + set_desc, set_x, set_y, set_z)
		server.applyChanges()

if __name__=="__main__":
	rospy.init_node("location_marker_node")

	server = InteractiveMarkerServer("location_markers")

	location_parser(read_wm_locations())

	location_update_status_server()
	location_update_confirm_server()

	rospy.spin()