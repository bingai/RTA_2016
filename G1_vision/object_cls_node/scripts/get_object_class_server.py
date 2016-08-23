#!/usr/bin/env python
import rospy
import rospkg
import sensor_msgs.msg
from object_cls_msgs.msg import BBox, BBoxList
from object_cls_msgs.srv import GetObjectClass, GetObjectClassRequest, GetObjectClassResponse

# import os.path
# import sys
# rospack = rospkg.RosPack()
# sys.path.insert(0, os.path.join(rospack.get_path("object_cls_node"), 'scripts'))
from object_classifier import DishClassifier, DishColorClassifier, DishDualClassifier

class GetObjectClassServer(object):

    def __init__(self):

        sv_ = rospy.Service('get_object_class', GetObjectClass, self.handle_get_object_class)

        self._task_names = { GetObjectClassRequest.TASK_TYPE_DISH: 'dish classifier',
                             GetObjectClassRequest.TASK_TYPE_FRIDGE: 'fridge object classifier' }

        self._task_type = GetObjectClassRequest.TASK_TYPE_DISH
        # self._classifer = DishClassifier()
        # self._classifer = DishColorClassifier()
        self._classifer = DishDualClassifier()

        rospy.loginfo("GetObjectClassServer Started.")

    def handle_get_object_class(self, req):

        resp = GetObjectClassResponse()
        resp.scores = []
        resp.predictions = []

        if req.task != self._task_type:
            rospy.loginfo("Error: wrong classification task type. The current server is running %s, but the client is requesting %s", self._task_names[self._task_type], self._task_names[req.task])
            return resp

        img_num = len(req.images)
        if img_num == 0:
            return resp
        if len(req.bbox_lists) != img_num:
            print "The length of the input lists do not match."
            return resp

        bbox_num = len(req.bbox_lists[0].bbox_list)
        if bbox_num == 0:
            return resp
        for bl in req.bbox_lists:
            if len(bl.bbox_list) != bbox_num:
                print "The length of the input bbox lists do not match."
                return resp

        images = req.images
        bbox_lists = [[b.box for b in bl.bbox_list] for bl in req.bbox_lists]

        rospy.loginfo("Processing %d images with %d bounding boxes." % (img_num, bbox_num))
        resp.scores, resp.predictions = self._classifer.classify_bbox_lists(images, bbox_lists)

        rospy.loginfo("Classification finished, sent response.")
        return resp


if __name__ == "__main__":

    rospy.init_node("get_object_class_server")
    server = GetObjectClassServer()
    rospy.spin()


