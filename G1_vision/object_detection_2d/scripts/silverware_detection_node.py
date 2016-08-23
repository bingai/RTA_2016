#!/usr/bin/env python
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
import sensor_msgs.msg
import shape_msgs.msg
import world_model_msgs.msg
import world_model_msgs.srv

class SilverwareDetectionNode:
    """
    Silverware Detection Node

    """
    def __init__(self):
        self.tf_listener_ = tf.TransformListener()

        self.camera_info_tp = '/topdown/camera/camera_info'
        # self.camera_info_tp = '/camera/rgb/camera_info'
        self.camera_sub_ = None
        self.result_pub_ = rospy.Publisher('silverwares', world_model_msgs.msg.Object, queue_size = 1)

        self.has_camera_info = False
        self.has_updated = False

        self.fix_z_ = -0.19
        self.bboxes_ = [[605, 216, 742, 465]]

    def extract_camera_info(self, camera_info):
        self.width = camera_info.width
        self.height = camera_info.height

        self.camera_matrix = np.asarray(camera_info.K).reshape((3,3))
        self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)

        self.has_camera_info = True

        rospy.loginfo("image size: %dx%d" % (self.width, self.height))
        # print 'camera matrix: ', camera_info.K


    def project_2d_to_3d(self, pt2d, trans, rotmat, fix_z):

        N = pt2d.shape[1]
        if pt2d.shape[0] == 2:
            pt2d = np.vstack((pt2d, np.ones((1, N))))

        pt1 = np.dot(np.dot(rotmat, self.camera_matrix_inv), pt2d)
        svec = (pt1[2,:] / (fix_z - trans[2])).reshape((1, N))

        pt3d = np.divide(pt1, np.repeat(svec, 3, axis=0)) + trans

        return pt3d

    def bbox_to_points(self, bbox):
        cx = (bbox[0]+bbox[2])/2
        cy = (bbox[1]+bbox[3])/2

        pt2d = [[bbox[0], bbox[1]],
                [bbox[0], bbox[3]],
                [bbox[2], bbox[3]],
                [bbox[2], bbox[1]],
                [cx, cy]]

        return np.asarray(pt2d).T

    def publish_det(self):
        if not self.has_camera_info:
            return

        try:
            (trans, rot) = self.tf_listener_.lookupTransform("/base", "/top_down_camera_link", rospy.Time(0))
            # print 'trans: ', trans
            # print 'rot: ', rotfix_z_

            rotmat = tf.transformations.quaternion_matrix(rot)
            rotmat = rotmat[:3, :3]
            trans = np.asarray(trans).reshape((3, 1))

            obj = world_model_msgs.msg.Object()
            obj.id = 'sw_dummy'
            obj.primitives = []
            obj.primitive_poses = []

            for i, bbox in enumerate(self.bboxes_):

                pt2d = self.bbox_to_points(bbox)
                pt3d = self.project_2d_to_3d(pt2d, trans, rotmat, self.fix_z_)
                xvec = pt3d[0,:]
                yvec = pt3d[0,:]

                shape = shape_msgs.msg.SolidPrimitive()
                shape.type = shape_msgs.msg.SolidPrimitive.BOX
                shape.dimensions = [max(xvec)-min(xvec), max(yvec)-min(yvec), 0.05]

                pose = geometry_msgs.msg.Pose()
                pose.position = geometry_msgs.msg.Point(pt3d[0,-1], pt3d[1,-1], pt3d[2,-1])
                pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)

                obj.primitives.append(shape)
                obj.primitive_poses.append(pose)

            if not self.has_updated:
                rospy.loginfo("Updating world model ...")
                objects_info = [obj]
                self.wm_client_update_states_objects(objects_info)


            self.result_pub_.publish(obj)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("No tf information")

    def wm_client_update_states_objects(self, objects_info):
        service_name = '/world_model/update_states_objects'
        try:
            rospy.wait_for_service(service_name, timeout=5.0)
        except rospy.exceptions.ROSException, e:
            rospy.loginfo("Timeout when waiting for service %s" % (service_name))
            return

        try:
            update_states_objects = rospy.ServiceProxy(service_name, world_model_msgs.srv.UpdateStatesObjects)
            resp = update_states_objects(objects_info, 0)
            if resp.success:
                self.has_updated = True
                rospy.loginfo("World model updated")
            else:
                rospy.loginfo("World model update failed (UpdateStatesObjects)")

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: (%s) %s" % (service_name, e))


    def spin(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if not self.has_camera_info:
                try:
                    camera_info = rospy.wait_for_message(self.camera_info_tp, sensor_msgs.msg.CameraInfo, timeout=1)
                    self.extract_camera_info(camera_info)
                except rospy.exceptions.ROSException, e:
                    rospy.loginfo('No camera info has been received')
                    pass

            if len(self.bboxes_) > 0:
                self.publish_det()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('silverware_detection_node')

    detector = SilverwareDetectionNode()
    detector.spin()

