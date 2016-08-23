#!/usr/bin/env python

import cv2
import numpy as np
import pytesseract as py_ocr
import PIL.Image as ocr_img_type
from sensor_msgs.msg import Image
import collections
import rospy
import rospkg
import os
from cv_bridge import CvBridge, CvBridgeError
from homography import find_lcd
from temp_smooth import return_smooth
from lcd_ocr.srv import *

bridge = CvBridge()

# PARAMS
crop_offset = collections.namedtuple('CropOffset','top, bottom, left, right')
crop_offset.top = 40
crop_offset.bottom = 160
crop_offset.left = 10
crop_offset.right = 10
tl = [0,0]
tr = [0,0]
bl = [0,0]
br = [0,0]

# VARIABLES
crop = 0
img1 = None # Hand Camera #
img2 = None # Template #
verify = False # Numpad Verification Result #
ts_buffer = None # Temporal Smoothing Buffer #
blah = 0 # Blah. #
numpad = 0 # Numpad OCR #

def get_cam(msg):
	global img1
	try:
		img1 = bridge.imgmsg_to_cv2(msg, "bgr8")
		img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
	except CvBridgeError, e:
		print(e)

def get_crop():
	global tl, bl, tr, br
	global img2
	global img1
	print 'GETTING LCD'
	for k in range(20):
		box = find_lcd(img2, img1)
		if box == None:
			pass
		else:
			tl = box[0]
			bl = box[1]
			br = box[2]
			tr = box[3]
	print 'FOUND LCD'

def displayOCR(to_check):
	global crop
	global blah
	global ts_buffer
	global verify
	global crop_offset
	global numpad
	global img2
	global img1
	global tl, tr, bl, br

	print 'VERIFYING INPUT: %d' % (to_check.num_input)
	
	ts_buffer = collections.deque(maxlen=10)
	frame_crop = img1

	if crop == 0:
		get_crop()
	frame_crop = frame_crop[tl[1]+crop_offset.top:bl[1]-crop_offset.bottom,tl[0]+crop_offset.left:tr[0]-crop_offset.right]
	print 'CROPPED LCD'
	frame_crop = cv2.medianBlur(frame_crop,5)
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
	frame_crop = cv2.erode(frame_crop,kernel,iterations = 2)
	# kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
	# frame_crop = cv2.dilate(frame_crop,kernel,iterations = 5)
	# h, w = frame_crop.shape
	# frame_crop = cv2.line(frame_crop,(w/2-7,0),(w/2-9,h),255,5)

	ret, frame_crop = cv2.threshold(frame_crop,100,255,cv2.THRESH_BINARY_INV)
	cv2.imshow('LCD_BIN',frame_crop)
	cv2.waitKey(1000)
	cv2.destroyAllWindows()

	ocr_img = ocr_img_type.fromarray(frame_crop)
	numpad = py_ocr.image_to_string(ocr_img, config="-l letsgodigital -psm 8 nobatch digits")
	print 'I SEE %d ON LCD' % (int(numpad))
	if int(numpad) == int(to_check.num_input):
		verify = True
	else:
		verify = False

	return verify

def verify_LCD(req):
	checker = displayOCR(req)
	if checker:
		print 'It\'s all gooood!'
	else:
		print 'Meh. That\'s bad!'
	return ocrResponse(checker)

def verify_LCD_server():
	global img2

	rp = rospkg.RosPack()
	template_path = os.path.join(rp.get_path("lcd_ocr"), "image", "template2.jpg")
	img2 = cv2.imread(template_path,0)

	rospy.init_node('verify_LCD_server')

	image_topic = '/cameras/right_hand_camera/image'
	rospy.Subscriber(image_topic, Image, get_cam)
	rospy.wait_for_message(image_topic,Image)
	
	s = rospy.Service('/LCD_OCR', ocr, verify_LCD)
	print "Waiting to verify"
	rospy.spin()

if __name__ == "__main__":
    verify_LCD_server()