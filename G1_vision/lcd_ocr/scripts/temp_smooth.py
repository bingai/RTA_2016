#!/usr/bin/env python

import collections

temp_image = None
ts_buffer = collections.deque(maxlen=10)

def return_smooth(img_buffer):
	global temp_image
	for x in range(len(img_buffer)):
		if x == 0:
			temp_image = img_buffer[x]
		else:
			temp_image = temp_image + img_buffer[x]
	temp_image = temp_image/len(img_buffer)
	return temp_image