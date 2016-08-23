#!/usr/bin/env python
import rospy
import rospkg
import sensor_msgs.msg
from object_cls_msgs.msg import BBox, BBoxList
from object_cls_msgs.srv import GetObjectClass, GetObjectClassRequest

import os.path
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import scipy.io as sio


class DummyGetObjectClassClient(object):

    def __init__(self):
        self.get_object_class = None
        service_name = 'get_object_class'
        self.service_name = service_name

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.exceptions.ROSException, e:
            rospy.loginfo("Timeout when waiting for service %s" % (service_name))
            return

        try:
            self.get_object_class = rospy.ServiceProxy(service_name, GetObjectClass)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: (%s) %s" % (service_name, e))

    def prepare_data(self):
        req = GetObjectClassRequest()

        rospack = rospkg.RosPack()
        data_dir = os.path.join(rospack.get_path("object_cls_node"), 'data')

        bridge = CvBridge()
        img = cv2.imread(os.path.join(data_dir, 'test01.jpg'), 1)
        req.images.append(bridge.cv2_to_imgmsg(img, encoding='bgr8'))
        img = cv2.imread(os.path.join(data_dir, 'test02.jpg'), 1)
        req.images.append(bridge.cv2_to_imgmsg(img, encoding='bgr8'))

        bbox = sio.loadmat(os.path.join(data_dir, 'test01_boxes.mat'))
        bbox = bbox['boxes']
        bbox_list_msg = BBoxList()
        for i in range(bbox.shape[0]):
            bbox_msg = BBox()
            bbox_msg.id = 'bbox_%04d' % i
            bbox_msg.box = bbox[i, :]
            bbox_list_msg.bbox_list.append(bbox_msg)
        req.bbox_lists.append(bbox_list_msg)

        bbox = sio.loadmat(os.path.join(data_dir, 'test02_boxes.mat'))
        bbox = bbox['boxes']
        bbox_list_msg = BBoxList()
        for i in range(bbox.shape[0]):
            bbox_msg = BBox()
            bbox_msg.id = 'bbox_%04d' % i
            bbox_msg.box = bbox[i, :]
            bbox_list_msg.bbox_list.append(bbox_msg)
        req.bbox_lists.append(bbox_list_msg)

        req.task = GetObjectClassRequest.TASK_TYPE_DISH

        return req

if __name__ == "__main__":

    dummy_client = DummyGetObjectClassClient()
    req = dummy_client.prepare_data()

    resp = dummy_client.get_object_class(req)
    print resp.scores
    print resp.predictions




