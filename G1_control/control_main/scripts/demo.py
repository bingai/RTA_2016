#! /usr/bin/env python
import rospy
import sys
import tf
import moveit_commander
from tf.transformations import *
import math
import copy
from baxter_arms import Arm
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from itertools import takewhile
import IPython
import time
import numpy as np
import numpy.linalg as la
from collections import defaultdict, deque
from itertools import izip
import moveit_msgs
from std_msgs.msg import String
from reflex_hand_controller import Reflex_hand_controller

def deg2rad(deg):
    return deg * math.pi / 180

def upright_angle(pose):
    q = quaternion_msg_to_tuple(pose.pose.orientation)
    z_axis = quaternion_matrix(q)[:3, 2]
    return angle_between(z_axis, np.array([0, 0, 1]))

def get_link_pose(tf_listener, robot, link_name, base = None, duration = 2):
        if base == None:
            base = robot.get_planning_frame()

        ########################################################################
        ## recover object pose from transform
        # try:
        tf_listener.waitForTransform(base, link_name, rospy.Time(0), rospy.Duration(duration))
        (trans, rot) = tf_listener.lookupTransformFull(base, rospy.Time(0), link_name, rospy.Time(0), base)
        pose = PoseStamped()
        pose.header.frame_id = base
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]

        if 'nakedbottle' in link_name:
            angle = upright_angle(pose)

            if deg2rad(10) < angle < deg2rad(80):
                pose.pose.orientation = quaternion_tuple_to_msg(quaternion_from_euler(0, 0, 0))

        # except:
        #     rospy.logerr('cannot find transform for %s' % link_name)
        #     raise Exception()

        return pose

def mean(data):
    return sum(data) / len(data)

def variance(data):
    m = mean(data)
    return sum((x - m)**2 for x in data) / len(data)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)

def quaternion_msg_to_tuple(msg):
    return (msg.x, msg.y, msg.z, msg.w)

def quaternion_tuple_to_msg(tuple):
    return Quaternion(tuple[0], tuple[1], tuple[2], tuple[3])






class Plan:
    def __init__(self, *steps):
        self.steps = []
        self.step_id = 0

        for step in steps:
            self.append_step(step)

    def get_last_motion_plan(self):
        for step_id, step in reversed(self.steps):
            for plan in reversed(step):
                if not hasattr(plan, '__call__'):
                    return plan
        return None

    def get_step(self):
        return self.step_id

    def set_step(self, step_id):
        self.step_id = step_id

    def append_step(self, step):
        self.steps.append((self.step_id, step))
        self.step_id += 1

    def revert_to_step(self, revert_id):
        if revert_id < 0 or revert_id > self.step_id:
            raise IndexError()

        self.steps = list(takewhile(lambda x: x[0] < revert_id, self.steps))
        self.step_id = revert_id

    def __len__(self):
        return sum(len(x) for _, x in self.steps)

    def __getitem__(self, i):
        if i < 0:
            i += len(self)
        if i < 0:
            raise IndexError('list index out of range')
        for step_id, step in self.steps:
            if i < len(step):
                return step[i]
            else:
                i -= len(step)
        raise IndexError('list index out of range')

class Execution:
    def __init__(self):
        self.tf_listener = tf.TransformListener()

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        #self.right_arm = Arm('right', self.robot, self.scene, self.tf_listener)
        self.left_hand = Reflex_hand_controller()

        #### Reset the hand fingers to parallel!!!!!!!

        self.left_arm = Arm('left', self.left_hand, self.robot, self.scene, self.tf_listener)

        #self.head_pub = rospy.Publisher('/head_image', String, queue_size=1, latch= True)

        self.models = {}
        self.models['ar_marker_1'] = (0.17, 0.17, 0.08)



#         self.speakable_name = {'ar_marker_1': 'bowl'}
#         self.speakable_name = defaultdict(lambda: 'object', self.speakable_name)
#
#
#         self.speak = rospy.ServiceProxy('voice', Voice)
#

        self.display_trajectory_publisher = rospy.Publisher(
                                                '/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory, queue_size=10)

        self.initialize_scene()

    def copy_pose(self, pose, dx = 0, dy = 0, dz = 0):
        new_pose = copy.deepcopy(pose)
        new_pose.pose.position.x += dx
        new_pose.pose.position.y += dy
        new_pose.pose.position.z += dz
        return new_pose

    def update_pose(self, pose, x = 0, y = 0, z = 0):
        new_pose = copy.deepcopy(pose)
        new_pose.pose.position.x = x
        new_pose.pose.position.y = y
        new_pose.pose.position.z = z
        return new_pose

    def reset_arms(self):
        self.left_arm.go2named_pose('observe_l')
        if query_yes_no('Calibrate?'):
            self.left_hand.fingers_calibrate()
        self.left_hand.tactile_calibrate()
        self.left_hand.init_shape()

        rospy.sleep(0.5)

    def get_object_class(self, object_id):
        object_id = object_id.lower()
        if 'ar_marker_1' in object_id:
            return 'ar_marker_1'
        raise Exception('No known class for object %s!' % object_id)

    def add_model2world(self, object_id, object_class, pose = None):
        # TODO assumes that model is a box, could have a mesh instead

        ########################################################################
        ## add model to planning scene
        model = self.models[object_class]

        try:
            if not pose:
                pose = get_link_pose(self.tf_listener, self.robot, object_id)
                pose.pose.position.z += model[2] / 2.0

            self.scene.remove_world_object(object_id)
            self.scene.add_box(object_id, pose, model)
        except:
            rospy.logerr('could not add %s to planning scene because tf not available. Continueing anyway...' % object_id)


    def update_planning_scene(self):
        frames = self.tf_listener.getFrameStrings()

        condition = lambda x: 'ar_marker_1' in x \
                              and x not in self.left_arm.attached_objects

        #condition = lambda x: 'nakedbottle' in x or 'mixing_bottle' in x

        objects = [x for x in frames if condition(x)]

        print objects

        for object_id in objects:
            object_class = self.get_object_class(object_id)
            self.add_model2world(object_id, object_class)

    def initialize_scene(self):
        # clean the scene
        self.scene.remove_world_object("table")

        # publish the table scene
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.72
        p.pose.position.y = 0
        p.pose.position.z = -0.68
        p.pose.orientation.w = 1.0
        self.scene.add_box("table", p, (0.8, 1.6, 1.0))


    def distance(self, pose1, pose2):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z

        return math.sqrt(dx**2 + dy**2 + dz**2)


    def rotate_pose(self, pose, r, p, y):
        q1 = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        q2 = quaternion_from_euler(r, p, y)
        q2 = quaternion_multiply(q1, q2)
        pose2 = self.copy_pose(pose)
        pose2.pose.orientation.x = q2[0]
        pose2.pose.orientation.y = q2[1]
        pose2.pose.orientation.z = q2[2]
        pose2.pose.orientation.w = q2[3]
        return pose2

    def plan_grasps(self, object_id, object_pose):

        pre_grasps = self.update_pose(self.left_arm.get_current_pose(),
                    x = object_pose.pose.position.x - 0.3,
                    y = object_pose.pose.position.y,
                    z = object_pose.pose.position.z + 0.14)

        pre_grasps = self.rotate_pose(pre_grasps, 0, 0, 1.57)

        return pre_grasps


    def plan_grasps2(self, object_id, object_pose):

        pre_grasps = self.update_pose(self.left_arm.get_current_pose(),
                    x = object_pose.pose.position.x - 0.3,
                    y = object_pose.pose.position.y,
                    z = object_pose.pose.position.z + 0.14)

        # pre_grasps = self.rotate_pose(pre_grasps, 0, 0, 1.57)

        return pre_grasps

    def plan_grasps3(self, object_id, object_pose):

        pre_grasps = self.update_pose(self.left_arm.get_current_pose(),
                    x = object_pose.pose.position.x,
                    y = object_pose.pose.position.y,
                    z = object_pose.pose.position.z + 0.4)

        pre_grasps = self.rotate_pose(pre_grasps, -1.57, 0, 0)

        return pre_grasps

    def plan_pick(self, arm, object_id, object_pose):
        #self.speak("Plan to pick the %s" %self.speakable_name[object_id])

        pre_grasps = self.plan_grasps(object_id, object_pose)
        print pre_grasps
        grip_pose = self.copy_pose(pre_grasps, dx = 0.12, dz = -0.03)

        rospy.loginfo('Planning pre grasp')
        arm.set_start_state_to_current_state()
        arm.clear_path_constraints()
        arm.set_pose_target(pre_grasps)

        # import pdb; pdb.set_trace()
        grasp_plan = arm.plan(tries=1)

        if grasp_plan == None:
            rospy.loginfo('Planning failed for grasp')

        ####################################################################
        ## Plan path to grasp position
        rospy.loginfo("### Planning grip pose ###")

        upright_orientation = [-0.0403406148832, 0.7015322435200, -0.0112477314665, 0.7114060968460]


        self.scene.remove_world_object(object_id)
        arm.set_start_state_to_end_of_plan(grasp_plan)
        #arm.lock_orientation(grip_pose, x_tolerance=0.1, y_tolerance=0.1, z_tolerance=0.1)
        arm.set_pose_target(grip_pose)

        grip_plan = arm.plan(tries=2)
        if grip_plan == None:
            rospy.logerr("Cannot reach forward to grasp object for this grasp!")
            self.initialize_scene()

        #self.initialize_scene()

        ####################################################################
        ## Plan path to slightly raised position
        rospy.loginfo("### Planning path to slightly raised position ###")

        pre_hold_pose = self.copy_pose(grip_pose, dz = 0.3)

        arm.set_start_state_to_end_of_plan(grip_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(pre_hold_pose)

        pre_hold_plan = arm.plan(tries=2)
        if pre_hold_plan == None:
            rospy.logerr("Cannot lift object in this position! Trying new grasp!")
            self.initialize_scene()

        #############################################
        ## Plan to pre place
        rospy.loginfo("### Planning path to pre place ###")
        pre_place_pose = self.copy_pose(pre_hold_pose, dx = 0.2)

        micro_pose = self.copy_pose(pre_hold_pose)
        micro_pose.pose.position.x = .31849
        micro_pose.pose.position.y = 1.0261
        micro_pose.pose.position.z = .13651
        micro_pose.pose.orientation.x = -.46467
        micro_pose.pose.orientation.y = -.4538
        micro_pose.pose.orientation.z = -.59912
        micro_pose.pose.orientation.w = -.46819

        #pre_place_pose = self.copy_pose(micro_pose)

        arm.set_start_state_to_end_of_plan(pre_hold_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(pre_place_pose)

        pre_place_plan = arm.plan(tries=5)

        #####################################
        ## plan to place
        rospy.loginfo("### Planning path to place ###")
        place_pose = self.copy_pose(pre_place_pose, dz = -0.3)
        #place_pose = self.copy_pose(pre_place_pose, dy = 0.02)
        arm.set_start_state_to_end_of_plan(pre_place_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(place_pose)

        place_plan = arm.plan(tries=5)

        #####################################
        ## plan to retreat
        rospy.loginfo("### Planning path to retreat ###")
        retreat_pose = self.copy_pose(place_pose, dz = 0.2, dx = - 0.3)
        #retreat_pose = self.copy_pose(place_pose, dy = -0.1, dx = 0.1)
        arm.set_start_state_to_end_of_plan(place_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(retreat_pose)

        retreat_plan = arm.plan(tries=5)

        full_plan = []
        full_plan.append(grasp_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(lambda: arm.hand.pre_grasp(2))
        full_plan.append(grip_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(lambda: raw_input("Closing?"))
        full_plan.append(lambda: arm.hand.close())
        full_plan.append(pre_hold_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(pre_place_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(place_plan)
        full_plan.append(lambda: raw_input("Open?"))
        full_plan.append(lambda: arm.hand.pre_open())
        full_plan.append(retreat_plan)
        #self.visualize_plan(full_plan)

        return full_plan

    def plan_pick2(self, arm, object_id, object_pose):
        #self.speak("Plan to pick the %s" %self.speakable_name[object_id])

        pre_grasps = self.plan_grasps2(object_id, object_pose)
        print pre_grasps
        grip_pose = self.copy_pose(pre_grasps, dx = 0.12, dz = -0.03)

        rospy.loginfo('Planning pre grasp')
        arm.set_start_state_to_current_state()
        arm.clear_path_constraints()
        arm.set_pose_target(pre_grasps)

        # import pdb; pdb.set_trace()
        grasp_plan = arm.plan(tries=1)

        if grasp_plan == None:
            rospy.loginfo('Planning failed for grasp')

        ####################################################################
        ## Plan path to grasp position
        rospy.loginfo("### Planning grip pose ###")

        upright_orientation = [-0.0403406148832, 0.7015322435200, -0.0112477314665, 0.7114060968460]


        self.scene.remove_world_object(object_id)
        arm.set_start_state_to_end_of_plan(grasp_plan)
        #arm.lock_orientation(grip_pose, x_tolerance=0.1, y_tolerance=0.1, z_tolerance=0.1)
        arm.set_pose_target(grip_pose)

        grip_plan = arm.plan(tries=2)
        if grip_plan == None:
            rospy.logerr("Cannot reach forward to grasp object for this grasp!")
            self.initialize_scene()

        #self.initialize_scene()

        ####################################################################
        ## Plan path to slightly raised position
        rospy.loginfo("### Planning path to slightly raised position ###")

        pre_hold_pose = self.copy_pose(grip_pose, dz = 0.3)

        arm.set_start_state_to_end_of_plan(grip_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(pre_hold_pose)

        pre_hold_plan = arm.plan(tries=2)
        if pre_hold_plan == None:
            rospy.logerr("Cannot lift object in this position! Trying new grasp!")
            self.initialize_scene()

        #############################################
        ## Plan to pre place
        rospy.loginfo("### Planning path to pre place ###")
        pre_place_pose = self.copy_pose(pre_hold_pose, dx = 0.2)

        micro_pose = self.copy_pose(pre_hold_pose)
        micro_pose.pose.position.x = .31849
        micro_pose.pose.position.y = 1.0261
        micro_pose.pose.position.z = .13651
        micro_pose.pose.orientation.x = -.46467
        micro_pose.pose.orientation.y = -.4538
        micro_pose.pose.orientation.z = -.59912
        micro_pose.pose.orientation.w = -.46819

        #pre_place_pose = self.copy_pose(micro_pose)

        arm.set_start_state_to_end_of_plan(pre_hold_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(pre_place_pose)

        pre_place_plan = arm.plan(tries=5)

        #####################################
        ## plan to place
        rospy.loginfo("### Planning path to place ###")
        place_pose = self.copy_pose(pre_place_pose, dz = -0.3)
        #place_pose = self.copy_pose(pre_place_pose, dy = 0.02)
        arm.set_start_state_to_end_of_plan(pre_place_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(place_pose)

        place_plan = arm.plan(tries=5)

        #####################################
        ## plan to retreat
        rospy.loginfo("### Planning path to retreat ###")
        retreat_pose = self.copy_pose(place_pose, dz = 0.2, dx = - 0.3)
        #retreat_pose = self.copy_pose(place_pose, dy = -0.1, dx = 0.1)
        arm.set_start_state_to_end_of_plan(place_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(retreat_pose)

        retreat_plan = arm.plan(tries=5)

        full_plan = []
        full_plan.append(grasp_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(lambda: arm.hand.pre_grasp(0))
        full_plan.append(grip_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(lambda: raw_input("Closing?"))
        full_plan.append(lambda: arm.hand.close())
        full_plan.append(pre_hold_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(pre_place_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(place_plan)
        full_plan.append(lambda: raw_input("Open?"))
        full_plan.append(lambda: arm.hand.pre_open())
        full_plan.append(retreat_plan)
        #self.visualize_plan(full_plan)

        return full_plan

    def plan_pick3(self, arm, object_id, object_pose):
        #self.speak("Plan to pick the %s" %self.speakable_name[object_id])

        pre_grasps = self.plan_grasps3(object_id, object_pose)
        print pre_grasps
        grip_pose = self.copy_pose(pre_grasps, dz = -0.2)

        rospy.loginfo('Planning pre grasp')
        arm.set_start_state_to_current_state()
        arm.clear_path_constraints()
        arm.set_pose_target(pre_grasps)

        # import pdb; pdb.set_trace()
        grasp_plan = arm.plan(tries=5)

        if grasp_plan == None:
            rospy.loginfo('Planning failed for grasp')

        ####################################################################
        ## Plan path to grasp position
        rospy.loginfo("### Planning grip pose ###")

        #upright_orientation = [-0.0403406148832, 0.7015322435200, -0.0112477314665, 0.7114060968460]


        self.scene.remove_world_object(object_id)
        arm.set_start_state_to_end_of_plan(grasp_plan)
        #arm.lock_orientation(grip_pose, x_tolerance=0.1, y_tolerance=0.1, z_tolerance=0.1)
        arm.set_pose_target(grip_pose)

        grip_plan = arm.plan(tries=2)
        if grip_plan == None:
            rospy.logerr("Cannot reach forward to grasp object for this grasp!")
            self.initialize_scene()

        #self.initialize_scene()

        ####################################################################
        ## Plan path to slightly raised position
        rospy.loginfo("### Planning path to slightly raised position ###")

        pre_hold_pose = self.copy_pose(grip_pose, dz = 0.3)

        arm.set_start_state_to_end_of_plan(grip_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(pre_hold_pose)

        pre_hold_plan = arm.plan(tries=2)
        if pre_hold_plan == None:
            rospy.logerr("Cannot lift object in this position! Trying new grasp!")
            self.initialize_scene()

        #############################################
        ## Plan to pre place
        rospy.loginfo("### Planning path to pre place ###")
        pre_place_pose = self.copy_pose(pre_hold_pose, dy = 0.2)

        # micro_pose = self.copy_pose(pre_hold_pose)
        # micro_pose.pose.position.x = .31849
        # micro_pose.pose.position.y = 1.0261
        # micro_pose.pose.position.z = .13651
        # micro_pose.pose.orientation.x = -.46467
        # micro_pose.pose.orientation.y = -.4538
        # micro_pose.pose.orientation.z = -.59912
        # micro_pose.pose.orientation.w = -.46819

        #pre_place_pose = self.copy_pose(micro_pose)

        arm.set_start_state_to_end_of_plan(pre_hold_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(pre_place_pose)

        pre_place_plan = arm.plan(tries=5)

        #####################################
        ## plan to place
        rospy.loginfo("### Planning path to place ###")
        place_pose = self.copy_pose(pre_place_pose, dz = -0.3)
        #place_pose = self.copy_pose(pre_place_pose, dy = 0.02)
        arm.set_start_state_to_end_of_plan(pre_place_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(place_pose)

        place_plan = arm.plan(tries=5)

        #####################################
        ## plan to retreat
        rospy.loginfo("### Planning path to retreat ###")
        retreat_pose = self.copy_pose(place_pose, dz = 0.2, dx = - 0.3)
        #retreat_pose = self.copy_pose(place_pose, dy = -0.1, dx = 0.1)
        arm.set_start_state_to_end_of_plan(place_plan)
        #arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(retreat_pose)

        retreat_plan = arm.plan(tries=5)

        full_plan = []
        full_plan.append(grasp_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(lambda: arm.hand.pre_grasp(3))
        full_plan.append(grip_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(lambda: raw_input("Closing?"))
        full_plan.append(lambda: arm.hand.close())
        full_plan.append(pre_hold_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(pre_place_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(place_plan)
        full_plan.append(lambda: raw_input("Open?"))
        full_plan.append(lambda: arm.hand.pre_open())
        full_plan.append(retreat_plan)
        #self.visualize_plan(full_plan)

        return full_plan

    def pick_and_place(self, object_id, execute = True):
        ########################################################################
        ## Choose arm to use
        try:
            object_pose = get_link_pose(self.tf_listener, self.robot, object_id)
        except:
            #self.speak("I'm afraid I don't see the %s anywere." % self.speakable_name[object_id])
            return

        #update_planning_scene()

        rospy.loginfo("use left arm")
        arm = self.left_arm


        #self.head_pub.publish('planning')
        rospy.sleep(0.1)

        ## horizontal grasp and Preshap = pi
        #full_motion_plan = self.plan_pick(arm, object_id, object_pose)

        ## horizontal grasp and Preshap = 0
        #full_motion_plan = self.plan_pick2(arm, object_id, object_pose)
        
        ## vertical grasp and Preshap = 2*pi/3
        full_motion_plan = self.plan_pick3(arm, object_id, object_pose)


        # self.visualize_plan(full_motion_plan)
        #self.head_pub.publish('executing')
        rospy.sleep(0.1)
        if execute:
            try:
                arm.move2poseslist(full_motion_plan, True)
            except:
                return

    def visualize_plan(self, plan):
        for step in plan:
            if step and not hasattr(step, '__call__'):
                display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                display_trajectory.trajectory_start = self.left_arm.get_plan_start_state(step)
                display_trajectory.trajectory.append(step)
                self.display_trajectory_publisher.publish(display_trajectory)
                query_yes_no('continue?')
        query_yes_no('done!')


def query_yes_no(question, default="no"):
        """Ask a yes/no question via raw_input() and return their answer.

        "question" is a string that is presented to the user.
        "default" is the presumed answer if the user just hits <Enter>.
            It must be "yes" (the default), "no" or None (meaning
            an answer is required of the user).

        The "answer" return value is True for "yes" or False for "no".
        """
        valid = {"yes": True, "y": True, "ye": True,
                "no": False, "n": False}
        if default is None:
            prompt = " [y/n] "
        elif default == "yes":
            prompt = " [Y/n] "
        elif default == "no":
            prompt = " [y/N] "
        else:
            raise ValueError("invalid default answer: '%s'" % default)

        while True:
            sys.stdout.write(question + prompt)
            choice = raw_input().lower()
            if default is not None and choice == '':
                return valid[default]
            elif choice in valid:
                return valid[choice]
            elif choice == "c":
                exit()
                raise RuntimeError("Cancel Planning!")
            else:
                sys.stdout.write("Please respond with 'yes' or 'no' "
                        "(or 'y' or 'n').\n")

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('baxter_demo', anonymous=True)

    execution = Execution()
    execution.update_planning_scene()
    execution.reset_arms()


    execution.pick_and_place('ar_marker_1')

    moveit_commander.roscpp_shutdown()
