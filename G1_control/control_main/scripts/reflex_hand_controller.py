#!/usr/bin/env python

# Code for a script using the ReFlex Takktile hand
# Note: you must connect a hand by running "roslaunch reflex reflex.launch" before you can run this script

from math import pi, cos

import rospy
from std_srvs.srv import Empty

from reflex_msgs.msg import Command
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import ForceCommand
from reflex_msgs.msg import Hand
from reflex_msgs.msg import FingerPressure
from reflex_msgs.srv import SetTactileThreshold, SetTactileThresholdRequest


hand_state = Hand()

class Reflex_hand_controller(object):
  def __init__(self):

    # Services can automatically call hand calibration
    self._calibrate_fingers = rospy.ServiceProxy('/reflex_takktile/calibrate_fingers', Empty)
    self._calibrate_tactile = rospy.ServiceProxy('/reflex_takktile/calibrate_tactile', Empty)

    # Services can set tactile thresholds and enable tactile stops
    self._enable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/enable_tactile_stops', Empty)
    self._disable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/disable_tactile_stops', Empty)
    self._set_tactile_threshold = rospy.ServiceProxy('/reflex_takktile/set_tactile_threshold', SetTactileThreshold)

    # This collection of publishers can be used to command the hand
    self._command_pub = rospy.Publisher('/reflex_takktile/command', Command, queue_size=1)
    self._pos_pub = rospy.Publisher('/reflex_takktile/command_position', PoseCommand, queue_size=1)
    self._vel_pub = rospy.Publisher('/reflex_takktile/command_velocity', VelocityCommand, queue_size=1)
    self._force_pub = rospy.Publisher('/reflex_takktile/command_motor_force', ForceCommand, queue_size=1)

    self.INIT_SHAPE = 1.8


    # Constantly capture the current hand state
    rospy.Subscriber('/reflex_takktile/hand_state', Hand, hand_state_cb)

    print "Hand initialized!!!"


  def fingers_calibrate(self):
    raw_input('start calibrating fingers:[Enter]')
    self._calibrate_fingers()
    raw_input('done![Enter]')

  def tactile_calibrate(self):
    self._calibrate_tactile()
    rospy.sleep(1)


  def init_shape(self):
    self._pos_pub.publish(PoseCommand(self.INIT_SHAPE, self.INIT_SHAPE, self.INIT_SHAPE, 0))
    rospy.sleep(1)

  def open(self, f1=0, f2=0, f3=0, preshape=0):
    self._pos_pub.publish(PoseCommand(f1, f2, f3, preshape))
    rospy.sleep(1)

  def hold_current_pose(self):
    global hand_state

    f = [0, 0, 0, 0]
    for i in range(4):
      f[i] = hand_state.motor[i].joint_angle

    print f
    self.open(f[0],f[1],f[2],f[3])


  #def close(self, f1=0.5, f2=0.5, f3=0.5, preshape=0):
    # self._enable_tactile_stops()
    # p1 = FingerPressure([500, 500, 500, 500, 500, 15, 15, 15, 15])
    # p2 = FingerPressure([500, 500, 500, 500, 500, 15, 15, 15, 15])
    # p3 = FingerPressure([500, 500, 500, 500, 500, 30, 30, 30, 30])
    # threshold = SetTactileThresholdRequest([p1, p2, p3])
    # self._set_tactile_threshold(threshold)
    # self._vel_pub.publish(VelocityCommand(f1, f2, f3, preshape))
    # rospy.sleep(4)
    # self._disable_tactile_stops()

    #self.hold_current_pose()
  def close(self):
    self._force_pub.publish(ForceCommand(f1=150,f2=150,f3=300))
    rospy.sleep(5)

  def pre_grasp(self, mode = 0):

    # mug #1 horizontal grasp with Preshape = 0
    if mode == 0:
      f1_f2 = 1
      f3 = 1
      reshape = 0
      self.open(f1_f2,f1_f2,f3, 0)

    # bowl #1 horizontal grasp with Preshape = pi
    elif mode == 1:
    	self.open(self.INIT_SHAPE,self.INIT_SHAPE,self.INIT_SHAPE,1.57)
    	f1_f2 = 1
    	f3 = 1
    	reshape = 1.57
    	self.open(f1_f2,f1_f2,f3, reshape)

    # bowl #2 and bowl #3 horizontal grasp with Preshape = pi
    elif mode ==2:
      self.open(self.INIT_SHAPE,self.INIT_SHAPE,self.INIT_SHAPE,1.57)
      f1_f2 = 0.5
      f3 = 1.4
      reshape = 1.57
      self.open(f1_f2,f1_f2,f3, reshape)

    # vertical grasp with Preshape = 2*pi/3
    else:
    #  self.open(self.INIT_SHAPE, self.INIT_SHAPE, self.INIT_SHAPE, 1.05)
      f1_f2 = 1
      f3 = 0.7
      reshape = 1.05
      self.open(f1_f2,f1_f2,f3, reshape)

  def pre_open(self, offset=[-1, -1, -1.5, 0]):
    preopen = [0, 0, 0, 0, 0]
    for i in range(4):
      preopen[i] = hand_state.motor[i].joint_angle + offset[i]
    self.open(preopen[0],preopen[1],preopen[2], preopen[3])


def hand_state_cb(data):
    global hand_state
    hand_state = data


if __name__ == '__main__':
    rospy.init_node('HandNode')
    action = Reflex_hand_controller()

    # calibrate need to be done once every week
    action.fingers_calibrate()

    action.tactile_calibrate()
    print "tactiles are calibrated!"

    action.init_shape()

    action.pre_grasp(3)
    print "Pregrasp"

    raw_input("Closing?")
    action.close()
    print "close"

    raw_input("pre-open?")
    #action.open(f1=1.5, f2=1.5, f3=2, preshape=1.57)
    action.pre_open()
    raw_input("open?")
    action.open()
