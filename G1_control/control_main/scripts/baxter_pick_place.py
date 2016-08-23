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
from voice_server.srv import *
from collections import defaultdict, deque
from itertools import izip
import moveit_msgs
from std_msgs.msg import String


ORANGE_BOTTLE = 'rvs_orangenakedbottle-exported'
PURPLE_BOTTLE = 'rvs_purplenakedbottle-berry-exported'
GREEN_BOTTLE  = 'rvs_greennakedbottle3-exported'
WHITE_BOTTLE  = 'rvs_whitenakedbottle1-exported'

class BottleDroppedException(Exception): pass

class FailedToGraspException(Exception): pass

class CannotSeeBottleException(Exception): pass

class BottleHasBeenMovedException(Exception): pass

class PlanningException(Exception): pass

PROMPT = True

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

        self.right_arm = Arm('right', self.robot, self.scene, self.tf_listener)
        self.left_arm = Arm('left', self.robot, self.scene, self.tf_listener)

        self.head_pub = rospy.Publisher('/head_image', String, queue_size=1, latch= True)

        self.models = {}
        self.models['naked_bottle'] = (0.065, 0.065, 0.25)
        self.models['mixing_bottle'] = (0.19, 0.19, 0.57)



        self.speakable_name = {'rvs_orangenakedbottle-exported' : 'orange bottle',
                               'rvs_purplenakedbottle-berry-exported' : 'purple bottle',
                               'rvs_greennakedbottle3-exported' : 'green bottle',
                               'rvs_whitenakedbottle1-exported' : 'white bottle'}
        self.speakable_name = defaultdict(lambda: 'object', self.speakable_name)


        self.speak = rospy.ServiceProxy('voice', Voice)
        self.display_trajectory_publisher = rospy.Publisher(
                                                '/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory)

        self.initialize_scene()

    def make_recipe(self, recipe):
        pour = self.pour_naked_juice
        try:
            compiled_recipe = compile(recipe, 'compiled_recipe', 'exec')
        except Exception as e:
            print e.message
            rospy.logerr("Failed to parse message!")

        exec(compiled_recipe)

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

    def reset_arms(self, reset_grippers=False):
        self.right_arm.go2named_pose('observe_r')
        self.left_arm.go2named_pose('observe_l')

        if reset_grippers:
            self.right_arm.gripper.calibrate()
            self.left_arm.gripper.calibrate()
        rospy.sleep(0.5)

    def get_object_class(self, object_id):
        object_id = object_id.lower()
        if 'nakedbottle' in object_id:
            return 'naked_bottle'
        elif 'mixing_bottle' in object_id:
            return 'mixing_bottle'
        raise Exception('No known class for object %s!' % object_id)

    def add_model2world(self, object_id, object_class, pose = None):
        # TODO assumes that model is a box, could have a mesh instead

        ########################################################################
        ## add model to planning scene
        model = self.models[object_class]

        try:
            if not pose:
                pose = get_link_pose(self.tf_listener, self.robot, object_id)

            self.scene.remove_world_object(object_id)
            self.scene.add_box(object_id, pose, model)
        except:
            rospy.logerr('could not add %s to planning scene because tf not available. Continueing anyway...' % object_id)


    def update_planning_scene(self):
        frames = self.tf_listener.getFrameStrings()

        condition = lambda x: 'rvs' in x \
                              and x not in self.right_arm.attached_objects \
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


    def make_gripper_checker(self, arm, grasping=False):
        def gripper_checker():
            print arm.gripper.force()
            if arm.gripper.force() < 10:
                rospy.sleep(1)
                print arm.gripper.force()
                if arm.gripper.force() < 10:
                    # IPython.embed()
                    rospy.logerr('Bottle not in gripper')
                    if grasping:
                        raise FailedToGraspException('Failed to pick up bottle!')
                    else:
                        raise BottleDroppedException('Bottle dropped!')
        return gripper_checker


    def make_bottle_checker(self, old_pose, object_id):
        def bottle_checker():
            rospy.sleep(0.5)
            try:
                new_pose = get_link_pose(self.tf_listener, self.robot, object_id)
            except:
                rospy.logerr('cannot see bottle anymore!!!')
                raise CannotSeeBottleException('cannot see bottle anymore!!!')

            x = self.distance(old_pose, new_pose)

            if x > 0.05:
                rospy.logerr('bottle has been moved!!!')
                raise BottleHasBeenMovedException('bottle has moved!!!')
            else:
                rospy.loginfo('bottle is still there =)')
        return bottle_checker

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

    def make_bottle_pose_checker(self, object_id, old_pose, length, threshold):
        xyz_history = deque()
        if old_pose: old_angle = upright_angle(old_pose)
        def checker():
            try:
                new_pose = get_link_pose(self.tf_listener, self.robot, object_id, duration=1./5.)
            except:
                rospy.logwarn('cannot see bottle!')
                xyz_history.clear()
                return False

            if (not old_pose) or (self.distance(old_pose, new_pose) > 0.06 or abs(upright_angle(new_pose) - old_angle) > deg2rad(80)):
                if len(xyz_history) < length:
                    xyz_history.append((new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z))
                    rospy.logwarn('gathering data...')
                else:
                    xyz_history.rotate(-1)
                    xyz_history[-1] = (new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z)

                    # check if the variance of each axis is below the threshold
                    x = [variance(data) for data in izip(*xyz_history)]
                    print x
                    if max(x) < threshold:
                        return True

            else:
                rospy.logwarn('no significant change in bottle position')
                return False

        return checker

    def ask_for_help(self, object_id, last_position, question=''):
        if question == '': question = 'Would you please move the %s to an easier position?' % self.speakable_name[object_id]
        self.speak(question)

        rate = rospy.Rate(5)
        checker = self.make_bottle_pose_checker(object_id, last_position, 30, 1e-5)

        while not rospy.is_shutdown():
            if checker():
                break

            rate.sleep()

        self.speak('thank you!')
        return True

    def test_pour(self, arm, object_id):
        self.add_model2world('rvs_mixing_bottle', 'mixing_bottle')
        self.scene.remove_world_object(object_id)
        initial_pose = arm.get_current_pose()
        mixing_bottle_pose = get_link_pose(self.tf_listener, self.robot, 'rvs_mixing_bottle')

        pouring_poses = [pose.grasp_pose for pose in arm.plan_pouring_poses(True)]

        q = [pouring_poses[0].pose.orientation.x, pouring_poses[0].pose.orientation.y, pouring_poses[0].pose.orientation.z, pouring_poses[0].pose.orientation.w]

        arm.lock_position()
        arm.set_orientation_target(q)
        orient_plan = arm.plan(tries=10)

        if not orient_plan:
            return

        new_step = []
        new_step.append(orient_plan)



        for pouring_pose in pouring_poses:
            arm.lock_orientation(pouring_poses[0])
            arm.set_start_state_to_end_of_plan(new_step[-1])
            arm.set_pose_target(pouring_pose)
            rospy.loginfo('### Planning to pouring pose... ###')
            pre_pour_plan = arm.plan()
            if pre_pour_plan:

                pose = arm.tf_transform(mixing_bottle_pose, pouring_pose, False)
                pouring_direction = -1 if pose.pose.position.y >=0 else 1
                q1 = [pouring_pose.pose.orientation.x, pouring_pose.pose.orientation.y, pouring_pose.pose.orientation.z, pouring_pose.pose.orientation.w]
                q2 = quaternion_from_euler(0, 0, pouring_direction * 130 * math.pi / 180)
                pouring_pose_down = quaternion_multiply(q1, q2)


                arm.set_start_state_to_end_of_plan(pre_pour_plan)
                arm.lock_position(pouring_pose)
                arm.set_orientation_target(pouring_pose_down)

                pour_down_plan = arm.plan(tries=2)
                if pour_down_plan == None:
                    rospy.logerr('cannot tilt bottle down to pour!')
                    continue

                ####################################################################
                ## Plan upward tilting motion
                rospy.loginfo('### Planning upward tilting motion... ###')

                arm.set_start_state_to_end_of_plan(pour_down_plan)
                arm.set_pose_target(pouring_pose)

                pour_up_plan = arm.plan(tries=2)
                if pour_up_plan == None:
                    rospy.logerr('cannot tilt bottle up to stop pouring!')
                    continue

                new_step.append(pre_pour_plan)
                new_step.append(pour_down_plan)
                new_step.append(lambda: rospy.sleep(0.5))
                new_step.append(pour_up_plan)

        # arm.lock_orientation()
        # arm.set_pose_target(initial_pose)
        # return_plan = arm.plan(tries=3)
        # if return_plan:
        #     new_step.append(return_plan)







        arm.move2poseslist(new_step)

    def plan_grasp_motion(self, arm, object_id, object_pose, pre_grasp_pose, grip_pose, bottle_needs_orienting):

        if arm==self.right_arm:
            object_pose.pose.position.y+=0.007
            pre_grasp_pose.pose.position.y+=0.007
            grip_pose.pose.position.y+=0.007

        arm.set_start_state_to_current_state()
        arm.clear_path_constraints()
        arm.set_pose_target(pre_grasp_pose)

        # import pdb; pdb.set_trace()
        grasp_plan = arm.plan(tries=1)

        if grasp_plan == None:
            rospy.loginfo('Planning failed for grasp')
            raise PlanningException("Pregrasp planning failed")

        ####################################################################
        ## Plan path to grasp position
        rospy.loginfo("### Planning grip pose ###")

        upright_orientation = [-0.0403406148832, 0.7015322435200, -0.0112477314665, 0.7114060968460]


        self.scene.remove_world_object(object_id)
        arm.set_start_state_to_end_of_plan(grasp_plan)
        arm.lock_orientation(grip_pose, x_tolerance=0.1, y_tolerance=0.1, z_tolerance=0.1)
        arm.set_pose_target(grip_pose)
        if bottle_needs_orienting: self.scene.remove_world_object("table")

        grip_plan = arm.plan(tries=2)
        if grip_plan == None:
            rospy.logerr("Cannot reach forward to grasp object for this grasp!")
            self.initialize_scene()
            raise PlanningException("Cannot reach forward!")


        ####################################################################
        ## Plan path to slightly raised position
        rospy.loginfo("### Planning path to slightly raised position ###")

        pre_hold_pose = self.copy_pose(grip_pose, dz = 0.3)

        arm.set_start_state_to_end_of_plan(grip_plan)
        arm.lock_orientation(pre_hold_pose)
        arm.set_pose_target(pre_hold_pose)

        pre_hold_plan = arm.plan(tries=2)
        if pre_hold_plan == None:
            rospy.logerr("Cannot lift object in this position! Trying new grasp!")
            self.initialize_scene()
            raise PlanningException('Cannot lift object in this position!')

        self.initialize_scene()

        ####################################################################
        ## Plan path to properly orient the bottle
        if bottle_needs_orienting:
            rospy.loginfo('### Planning path to properly orient the bottle ###')

            arm.set_start_state_to_end_of_plan(pre_hold_plan)
            arm.lock_position(pre_hold_pose, radius=0.10)
            arm.set_orientation_target(upright_orientation)

            orient_plan = arm.plan(tries=2)
            if orient_plan == None:
                rospy.logerr("Cannot orient bottle! Trying new grasp!")
                raise PlanningException('Cannot orient bottle')

        full_plan = []
        full_plan.append(grasp_plan)
        full_plan.append(self.make_bottle_checker(object_pose, object_id))
        full_plan.append(grip_plan)
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(lambda: arm.gripper.close())
        full_plan.append(lambda: rospy.sleep(0.5))
        full_plan.append(self.make_gripper_checker(arm, grasping=True))
        full_plan.append(pre_hold_plan)
        full_plan.append(self.make_gripper_checker(arm))
        if bottle_needs_orienting: full_plan.append(orient_plan)

        # self.visualize_plan(full_plan)

        return full_plan

    def plan_extended_grasp(self, arm, grasp_instructions):
        full_plan = []
        rospy.logwarn('plan: ')
        for x in grasp_instructions.grasp_list:
            rospy.logwarn(x.grasps[0].object_id)


        # query_yes_no('wait')

        for grasps in (x.grasps for x in grasp_instructions.grasp_list):
            ####################################################################
            # Plan pick up motion
            object_id = grasps[0].object_id
            suggested_placement = grasps[0].suggested_placement
            object_class = self.get_object_class(object_id)

            rospy.loginfo("Removing %s..." % object_id)
            self.speak("Move the %s away" %self.speakable_name[object_id])


            # get bottle pose
            bottle_pose = get_link_pose(self.tf_listener, self.robot, object_id)

            # determine if the bottle is lying down
            q = quaternion_msg_to_tuple(bottle_pose.pose.orientation)
            z_axis = quaternion_matrix(q)[:3, 2]
            angle = angle_between(z_axis, np.array([0, 0, 1]))

            # bottle_needs_orienting: true if bottle is lying down, false otherwise
            if angle > math.pi / 4:
                rospy.logwarn('bottle is down')
                bottle_needs_orienting = True
            else:
                rospy.logwarn('bottle is up')
                bottle_needs_orienting = False

            success = False

            grasps.sort(key=lambda grasp: grasp.grasp_quality, reverse = True)
            
            for i, grasp in enumerate(grasps):
                rospy.loginfo("trying grasp %s..." % i)
                try:
                    arm.set_start_state_to_current_state()
                    # grasp_plan: [motion_plan, gripper_command, motion_plan, ...]

                    grasp_pose = grasp.pre_grasp_pose
                    pose_ref_grasp = arm.tf_transform(bottle_pose, grasp_pose, False)
                    pose_ref_grasp.pose.position.z -= 0.07
                    pose_ref_grasp.pose.position.x -= 0.02

                    grip_pose = arm.tf_transform(pose_ref_grasp, grasp_pose, True)
                    grip_pose.pose.orientation = grasp_pose.pose.orientation
                    grip_pose.header.frame_id = '/base'
                    
                    # grip_pose = grasp.grasp_pose

                    grasp_plan = self.plan_grasp_motion(arm, object_id, bottle_pose,
                                             grasp.pre_grasp_pose, grip_pose,
                                             bottle_needs_orienting)
                    # query_yes_no('wait')
                except PlanningException:
                    continue

                # import pdb; pdb.set_trace()
                # IPython.embed()

                ####################################################################
                # Plan motion to place bottle on safe location

                # plan motion to place bottle down
                last_plan = Plan(grasp_plan).get_last_motion_plan()
                orientations = []
                orientations.append(Quaternion(suggested_placement.orientation.x,suggested_placement.orientation.y,suggested_placement.orientation.z, suggested_placement.orientation.w))
                orientations.append(Quaternion(-0.0403406148832, 0.7015322435200, -0.0112477314665, 0.7114060968460))
                sign = (-1 if arm == self.right_arm else 1)
                orientations.extend(quaternion_tuple_to_msg(quaternion_from_euler(roll, math.pi / 2, 0))
                    for roll in np.arange(0, sign * math.pi / 2, sign * math.pi / 2 / 5))

                for i, orientation in enumerate(orientations):
                    arm.set_start_state_to_end_of_plan(last_plan)
                    hold_pose = PoseStamped()
                    hold_pose.header.frame_id = '/base'
                    hold_pose.pose = copy.deepcopy(suggested_placement)
                    hold_pose.pose.position.z += 0.2
                    hold_pose.pose.orientation = orientation

                    arm.lock_orientation(grasp.grasp_pose)
                    arm.set_pose_target(hold_pose)

                    print hold_pose.pose

                    place_plan_1 = arm.plan()
                    if not place_plan_1:
                        rospy.logerr("Cannot move %s to place for %sth rotation" % (object_id, i))
                        continue


                    arm.set_start_state_to_end_of_plan(place_plan_1)
                    place_pose = PoseStamped()
                    place_pose.header.frame_id = '/base'
                    place_pose.pose = copy.deepcopy(suggested_placement)
                    if (suggested_placement.position.z == -0.03):
                        place_pose.pose.position.y *= -1
                        place_pose.pose.position.z = -0.08
                    place_pose.pose.orientation = orientation

                    print place_pose.pose

                    arm.set_pose_target(place_pose)
                    place_plan_2 = arm.plan(2)

                    arm.clear_path_constraints()
                    if not place_plan_2:
                        rospy.logerr("Cannot place %s for %sth rotation" % (object_id, i))
                        continue
                    else:
                        break

                if not place_plan_2:
                    rospy.logerr("Cannot place %s for every orientation" % object_id)
                    continue
                
                place_plan = []
                place_plan.append(place_plan_1)
                place_plan.append(place_plan_2)

                # query_yes_no('wait')

                # since we only set a position (xyz) target do forward kinematics
                # to get full pose information
                robot_state = arm.get_plan_end_state(place_plan_2)
                place_pose = arm.get_position_fk(robot_state)

                # exit pose should be a backwards translation of place_pose
                # exit_pose = self.copy_pose(place_pose)
                # exit_pose = arm.tf_transform(exit_pose, place_pose)
                # exit_pose.pose.position.z -= 0.15
                # exit_pose = arm.tf_transform(exit_pose, place_pose, True)
                # exit_pose.pose.orientation = place_pose.pose.orientation
                # exit_pose.header.frame_id = '/base'

                # plan motion to retract gripper from bottle
                arm.set_start_state_to_end_of_plan(place_plan_2)
                arm.set_pose_target(hold_pose)
                exit_plan = arm.plan()
                if not exit_plan:
                    rospy.logerr("cannot exit")
                    continue
                # query_yes_no('wait')
                # new_bottle_pose is where the bottle will be placed in the planning scene
                new_bottle_pose = self.rotate_pose(place_pose, 0, math.pi / 2.0, 0)
                new_bottle_pose.pose.position.z -= 0.02

                arm.set_start_state_to_end_of_plan(exit_plan)
                arm.clear_path_constraints()
                arm.set_named_pose_target('observe_' + arm.side[0])
                self.add_model2world(object_id, object_class, new_bottle_pose)
                prev_planning_time = arm.get_planning_time()
                arm.set_planning_time(5)

                # plan motion back to ready position
                ready_position_plan = arm.plan(tries=5)
                arm.set_planning_time(prev_planning_time)
                if not ready_position_plan:
                    rospy.logerr("cannot plan ready position")
                    continue
                # query_yes_no('wait')
                success = True
                # import pdb; pdb.set_trace()
                break

            if not success:
                rospy.logerr("Cannot grasp %s"  % object_id)
                raise PlanningException("Cannot grasp %s" % object_id)

            full_plan.extend(grasp_plan)
            full_plan.extend(place_plan)
            full_plan.append(lambda: rospy.sleep(0.5))
            full_plan.append(lambda: arm.gripper.open())
            full_plan.append(lambda: rospy.sleep(0.5))
            full_plan.append(exit_plan)
            full_plan.append(ready_position_plan)

        return full_plan

    def plan_pour_naked_juice(self, arm, object_id, object_pose, pour_time):
        self.speak("Plan to pour the %s" %self.speakable_name[object_id])

        object_class = self.get_object_class(object_id)

        mixing_bottle_pose = get_link_pose(self.tf_listener, self.robot, 'rvs_mixing_bottle')
        q = quaternion_msg_to_tuple(object_pose.pose.orientation)

        z_axis = quaternion_matrix(q)[:3, 2]

        angle = angle_between(z_axis, np.array([0, 0, 1]))
        if angle > math.pi / 4:
            rospy.logwarn('bottle is down')
            bottle_needs_orienting = True
        else:
            rospy.logwarn('bottle is up')
            bottle_needs_orienting = False


        observe_pose = arm.get_current_pose()
        self.update_planning_scene()

        self.scene.remove_world_object(object_id)
        grasps = [pose.pre_grasp_pose for pose in arm.plan_grasps(object_id, object_class)]
        self.add_model2world(object_id, object_class, object_pose)

        # query_yes_no("wait")

        full_motion_plan = Plan()
        self.update_planning_scene()

        # import pdb; pdb.set_trace()

        if len(grasps) == 0:
            grasp_instructions = arm.plan_unblock(object_id, object_class)
            unblock_step = self.plan_extended_grasp(arm, grasp_instructions)
            if unblock_step:
                full_motion_plan.append_step(unblock_step)
                grasps = [pose.pre_grasp_pose for pose in arm.plan_grasps(object_id, object_class)]
            else:
                raise PlanningException("Cannot grasp object! Object is out of reach or is blocked! Please help!")
        else:
            unblock_step = []
            unblock_step.append(lambda: rospy.sleep(0.1))
            full_motion_plan.append_step(unblock_step)

        # start planning pouring poses
        pouring_grasps = arm.plan_pouring_poses(True)
        pouring_poses = [pose.grasp_pose for pose in pouring_grasps]

        # translate object from global frame to pregrasp frame, last flag specifies if it is inverse transform

        # first_pour_step = True
        grasp_idx = -1
        pouring_idx = -1

        UNBLOCK_STEP, GRASP_STEP, PREPOUR_STEP, POUR_STEP, REPLACE_STEP, RETRY_LAST_STEP = range(6)
        while not rospy.is_shutdown():
            ########################################################################
            ## Plan full grasp
            ########################################################################
            if full_motion_plan.get_step() == GRASP_STEP:
                ####################################################################
                ## Plan path to pregrasp position
                rospy.loginfo("### Planning pregrasp %s ###" % (grasp_idx + 1))

                full_motion_plan.revert_to_step(GRASP_STEP)                

                while True:
                    grasp_idx += 1
                    if grasp_idx >= len(grasps) or grasp_idx > 8:
                        raise PlanningException("Cannot grasp object! Object is out of reach or is blocked! Please help!")

                    grasp_pose = grasps[grasp_idx]
                    pose_ref_grasp = arm.tf_transform(object_pose, grasp_pose, False)
                    pose_ref_grasp.pose.position.z -= 0.07
                    pose_ref_grasp.pose.position.x -= 0.02

                    grip_pose = arm.tf_transform(pose_ref_grasp, grasp_pose, True)
                    grip_pose.pose.orientation = grasp_pose.pose.orientation
                    grip_pose.header.frame_id = '/base'
                    try:
                        new_step = self.plan_grasp_motion(arm, object_id, object_pose, grasp_pose, grip_pose, bottle_needs_orienting)
                        break
                    except PlanningException:
                        rospy.logerr("PlanningException in grasp planning!")

                full_motion_plan.append_step(new_step)

            ########################################################################
            ## Plan position from which to pour
            ########################################################################
            elif full_motion_plan.step_id == PREPOUR_STEP:
                ####################################################################
                ## Plan path to pouring pose
                rospy.loginfo('### Planning path to pouring pose ###')
                # if first_pour_step:
                #     arm.grasp_client.wait_for_result()
                #     pouring_poses = [pose.grasp_pose for pose in arm.grasp_client.get_result().grasps]
                #     first_pour_step = False

                full_motion_plan.revert_to_step(PREPOUR_STEP)
                arm.set_start_state_to_end_of_plan(full_motion_plan.get_last_motion_plan())

                plan = None
                while True:
                    pouring_idx += 1
                    if pouring_idx >= len(pouring_poses):
                        rospy.logerr('cannot complete task from this pouring configuration!')
                        pouring_idx = 0
                        full_motion_plan.set_step(GRASP_STEP)
                        break
                    pouring_pose = pouring_poses[pouring_idx]

                    arm.lock_orientation(pouring_pose)
                    arm.set_pose_target(pouring_pose)

                    pre_pour_plan = arm.plan(tries=1)
                    # query_yes_no("")
                    if pre_pour_plan != None:
                        break

                if pre_pour_plan == None:
                    continue

                new_step = []
                new_step.append(self.make_gripper_checker(arm))
                new_step.append(pre_pour_plan)

                full_motion_plan.append_step(new_step)

            ########################################################################
            ## Plan the motion that will actually pour the liquid
            ########################################################################
            elif full_motion_plan.get_step() == POUR_STEP:
                ####################################################################
                ## Plan downward tilting motion
                rospy.loginfo('### Planning downward tilting motion ###')
                pouring_angle = 128 if bottle_needs_orienting else 130

                pose = arm.tf_transform(mixing_bottle_pose, pouring_pose, False)
                pouring_direction = -1 if pose.pose.position.y >=0 else 1
                pouring_angle = 128 if bottle_needs_orienting else 130
                q1 = [pouring_pose.pose.orientation.x, pouring_pose.pose.orientation.y, pouring_pose.pose.orientation.z, pouring_pose.pose.orientation.w]
                q2 = quaternion_from_euler(0, 0, pouring_direction * pouring_angle * math.pi / 180)
                pouring_pose_down = quaternion_multiply(q1, q2)

                full_motion_plan.revert_to_step(POUR_STEP)
                arm.set_start_state_to_end_of_plan(full_motion_plan.get_last_motion_plan())
                arm.clear_path_constraints()
                arm.lock_position(pouring_pose)
                arm.set_orientation_target(pouring_pose_down)

                pour_down_plan = arm.plan(tries=2)
                if pour_down_plan == None:
                    rospy.logerr('cannot tilt bottle down to pour!')
                    full_motion_plan.set_step(PREPOUR_STEP)
                    continue

                ####################################################################
                ## Plan upward tilting motion
                rospy.loginfo('### Planning upward tilting motion... ###')

                arm.set_start_state_to_end_of_plan(pour_down_plan)
                arm.set_pose_target(pouring_pose)

                pour_up_plan = arm.plan(tries=2)
                if pour_up_plan == None:
                    rospy.logerr('cannot tilt bottle up to stop pouring!')
                    full_motion_plan.set_step(PREPOUR_STEP)
                    continue

                new_step = []
                new_step.append(self.make_gripper_checker(arm))
                new_step.append(pour_down_plan)
                new_step.append(self.make_gripper_checker(arm))
                new_step.append(lambda: rospy.sleep(pour_time))
                new_step.append(self.make_gripper_checker(arm))
                new_step.append(pour_up_plan)

                full_motion_plan.append_step(new_step)

            ########################################################################
            ## Plan placing the bottle back down on the table
            ########################################################################
            elif full_motion_plan.get_step() == REPLACE_STEP:
                ####################################################################
                ## Plan path back to hold pose
                rospy.loginfo('### Planning path back to hold pose... ###')
                orientations = []
                if not bottle_needs_orienting:
                    orientations.append(grip_pose.pose.orientation)

                orientations.append(Quaternion(-0.0403406148832, 0.7015322435200, -0.0112477314665, 0.7114060968460))
                sign = -1 if arm == self.right_arm else 1
                orientations.extend(quaternion_tuple_to_msg(quaternion_from_euler(roll, math.pi / 2, 0))
                                        for roll in np.arange(0, sign * math.pi / 2, sign * math.pi / 2 / 5))

                for i, orientation in enumerate(orientations):
                    rospy.loginfo('trying %sth orientation' % i)

                    hold_pose = self.copy_pose(grip_pose, dz = 0.35)
                    hold_pose.pose.orientation = orientation

                    pick_pose = self.copy_pose(hold_pose)
                    pick_pose.pose.position.z = -0.07

                    exit_pose = self.copy_pose(pick_pose)

                    exit_pose = arm.tf_transform(exit_pose, pick_pose)
                    exit_pose.pose.position.z -= 0.15
                    exit_pose = arm.tf_transform(exit_pose, pick_pose, True)
                    exit_pose.pose.orientation = pick_pose.pose.orientation
                    exit_pose.header.frame_id = '/base'

                    full_motion_plan.revert_to_step(REPLACE_STEP)
                    arm.set_start_state_to_end_of_plan(full_motion_plan.get_last_motion_plan())
                    arm.lock_orientation(hold_pose)

                    plans = arm.plan_to_poses(2, hold_pose, pick_pose, exit_pose)
                    if plans == None:
                        rospy.logerr('cannot replace_bottle!')
                    else:
                        rospy.loginfo('success!')
                        break

                if plans == None:
                    raise PlanningException('cannot replace bottle!')
                    full_motion_plan.set_step(PREPOUR_STEP)
                    continue

                hold_plan, pick_plan, exit_plan = plans

                ####################################################################
                ## Plan path back to the ready position
                rospy.loginfo('### Planning path to ready position ###')

                bottle_pose = self.rotate_pose(pick_pose, 0, math.pi / 2.0, 0)
                bottle_pose.pose.position.z -= 0.02

                arm.set_start_state_to_end_of_plan(exit_plan)
                arm.clear_path_constraints()
                arm.set_named_pose_target('observe_' + arm.side[0])
                self.add_model2world(object_id, object_class, bottle_pose)

                prev_planning_time = arm.get_planning_time()

                arm.set_planning_time(10)

                ready_position_plan = arm.plan(tries=5)
                if ready_position_plan == None:
                    rospy.logerr('Failed to plan to initial position, but accepting plan anyway')

                arm.set_planning_time(prev_planning_time)

                new_step = []
                new_step.append(self.make_gripper_checker(arm))
                new_step.append(hold_plan)
                new_step.append(self.make_gripper_checker(arm))
                new_step.append(pick_plan)
                new_step.append(self.make_gripper_checker(arm))
                new_step.append(lambda: rospy.sleep(0.5))
                new_step.append(lambda: arm.gripper.open())
                new_step.append(lambda: rospy.sleep(0.5))
                new_step.append(exit_plan)
                if ready_position_plan: new_step.append(ready_position_plan)

                full_motion_plan.append_step(new_step)
                break


            elif full_motion_plan.get_step() == RETRY_LAST_STEP:
                exit_pose.pose.position.z =+ 0.4

                arm.set_start_state_to_end_of_plan(full_motion_plan.get_last_motion_plan())
                arm.clear_path_constraints()
                old_position_tolerance = arm.get_goal_position_tolerance()
                arm.set_goal_position_tolerance(0.03)
                old_orientation_tolerance = arm.get_goal_orientation_tolerance()
                arm.set_goal_orientation_tolerance(0.15)
                arm.set_pose_target(exit_pose)


                recover_plan = arm.plan(tries=10)
                if not recover_plan:
                    rospy.logerr('Failed to recover! Ending planning')
                    break

                arm.set_start_state_to_end_of_plan(recover_plan)
                arm.set_goal_position_tolerance(old_position_tolerance)
                arm.set_goal_orientation_tolerance(old_orientation_tolerance)
                arm.set_named_pose_target('observe_' + arm.side[0])

                ready_position_plan = arm.plan(tries=5)
                if not ready_position_plan:
                    rospy.logerr('Still cannot plan to initial position! Ending planning')
                    break

                new_step = []
                new_step.append(recover_plan)
                new_step.append(ready_position_plan)

                full_motion_plan.append_step(new_step)

                break






        # IPython.embed()
        return full_motion_plan



    def pour_naked_juice(self, object_id, pour_time = 2, execute = True):
        ########################################################################
        ## Choose arm to use
        try:
            object_pose = get_link_pose(self.tf_listener, self.robot, object_id)
        except:
            self.speak("I'm afraid I don't see the %s anywere." % self.speakable_name[object_id])
            if self.ask_for_help(object_id, None, question = 'Could you put the %s back on the table' % self.speakable_name[object_id]):
                self.pour_naked_juice(object_id, pour_time, execute)
            return

        if object_pose.pose.position.y < 0:
            rospy.loginfo("use right arm")
            arm = self.right_arm
        else:
            rospy.loginfo("use left arm")
            arm = self.left_arm

        arm.gripper.open()

        self.head_pub.publish('planning')
        rospy.sleep(0.1)

        try:
            full_motion_plan = self.plan_pour_naked_juice(arm, object_id, object_pose, pour_time)
            retry = False
        except PlanningException:
            rospy.logwarn('Planning with first arm failed! trying other arm')
            retry = True
            self.speak("I don't think I can pick up the %s with my %s hand. Let me try my other hand." % (self.speakable_name[object_id], 'right' if arm == self.right_arm else 'left'))

        if retry:
            arm = self.left_arm if arm == self.right_arm else self.right_arm
            try:
                full_motion_plan = self.plan_pour_naked_juice(arm, object_id, object_pose, pour_time)
            except PlanningException:
                self.speak("I can't seem to grasp the %s" % self.speakable_name[object_id])
                if self.ask_for_help(object_id, object_pose):
                    self.pour_naked_juice(object_id, pour_time, execute)

                return



        # self.visualize_plan(full_motion_plan)
        self.head_pub.publish('executing')
        rospy.sleep(0.1)
        if execute:
            try:
                arm.move2poseslist(full_motion_plan, False)
            except BottleDroppedException:
                self.speak("oops, I dropped the bottle. Let me try again")
                self.pour_naked_juice(object_id, pour_time)
            except FailedToGraspException:
                self.speak("clumsy me, let me try again.")
                self.pour_naked_juice(object_id, pour_time)
            except BottleHasBeenMovedException:
                self.speak("oops, the bottle isn't were I thought it was, let me try again")
                self.pour_naked_juice(object_id, pour_time)
            except CannotSeeBottleException:
                self.speak("I can't see the bottle anymore!")
                if self.ask_for_help(object_id, object_pose, question = 'Could you put the %s back on the table' % self.speakable_name[object_id]):
                    self.pour_naked_juice(object_id, pour_time, execute)

    def visualize_plan(self, plan):                   
        for step in plan:
            if step and not hasattr(step, '__call__'):
                display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                display_trajectory.trajectory_start = self.right_arm.get_plan_start_state(step)
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
    rospy.init_node('baxter_pick_place_test', anonymous=True)

    execution = Execution()    
    execution.update_planning_scene()
    execution.reset_arms(reset_grippers=True)

    # IPython.embed()
    # arm = execution.left_ar
    # current_pose = arm.get_current_pose()
    # # arm.lock_orientation(current_pose)
    # current_pose.pose.position.x = 0.493599878888
    # current_pose.pose.position.y = -0.0444512320684
    # current_pose.pose.position.z = -0.0840730727952
    # current_pose.pose.orientation.x = -0.51815
    # current_pose.pose.orientation.y = 0.49932
    # current_pose.pose.orientation.z = 0.46041
    # current_pose.pose.orientation.w = 0.51982

    # arm.set_pose_target( current_pose)
    # arm.plan()

    # "orange" : "/rvs_orangenakedbottle-exported",
    # "purple" : "rvs_purplenakedbottle-berry-exported",
    # "green" : "/rvs_greennakedbottle3-exported",
    # "white" : "/rvs_whitenakedbottle1-exported",
    # object_id = 'fake_nakedbottle'
    # recipe = "pour('%s', 1)" % object_id
    #recipe = "pour('kvs_greennakedbottle3-exported', 2)\npour('kvs_orangenakedbottle-exported', 0.5)"

    # execution.pour_naked_juice('rvs_whitenakedbottle1-exported')
    # execution.pour_naked_juice('rvs_orangenakedbottle-exported')
    # execution.left_arm.gripper.force = lambda: 40
    # execution.right_arm.gripper.force = lambda: 40
    
    # execution.pour_naked_juice("rvs_purplenakedbottle3")
    # execution.pour_naked_juice('rvs_greennakedbottle3-exported')

    # execution.pour_naked_juice(ORANGE_BOTTLE)


    #print recipe
    # exe = execution
    # exe.update_planning_scene()

    # IPython.embed()



    #execution.make_recipe(recipe)
    # execution.test_pour(execution.right_arm, GREEN_BOTTLE)


    moveit_commander.roscpp_shutdown()
