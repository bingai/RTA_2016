#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from reflex_msgs.msg import Hand

from reflex_msgs.msg import Command
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import ForceCommand

from std_srvs.srv import Empty

import sys, select, termios, tty

msg = """
Control Your Turtlebot!
---------------------------
Finger control: open | close
   f1    u    i
   f2    j    k
   f3    m    ,
   reshape p  ;

q/z : increase/decrease open/close f1 speed by 10%
w/x : increase/decrease open/close f2 by 10%
e/c : increase/decrease open/close f3 by 10%
space key : stop
a or CTRL-C: quit
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'u':(1,0, 0, 0),
        'i':(-1, 0, 0, 0),
        'j':(0,1, 0, 0),
        'k':(0,-1, 0, 0),
        'm':(0, 0,1, 0),
        ',':(0, 0,-1, 0),
        'p':(0, 0, 0 , 1),
        ';':(0, 0, 0 , -1),
        }

speedBindings={
        'q':(1.1,1 ,1),
        'z':(.9,1,1),
        'w':(1,1.1,1),
        'x':(1,.9,1),
        'e':(1,1,1.1),
        'c':(1,1,.9),
          }

hand_state = Hand()

def hand_state_cb(data):
    global hand_state
    hand_state = data

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(f1_spd, f2_spd, f3_spd):
    return "currently:\tf1_angle %s\tf1_spd %s\n\tf2_angle %s\tf2_spd %s\n\tf3_angle %s\tf3_spd %s" % (hand_state.motor[0].joint_angle,f1_spd,hand_state.motor[1].joint_angle,f2_spd,hand_state.motor[2].joint_angle,f3_spd)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot_teleop')

    calibrate_fingers = rospy.ServiceProxy('/reflex_takktile/calibrate_fingers', Empty)
    calibrate_tactile = rospy.ServiceProxy('/reflex_takktile/calibrate_tactile', Empty)

    command_pub = rospy.Publisher('/reflex_takktile/command', Command, queue_size=1)
    pos_pub = rospy.Publisher('/reflex_takktile/command_position', PoseCommand, queue_size=1)
    vel_pub = rospy.Publisher('/reflex_takktile/command_velocity', VelocityCommand, queue_size=1)
    force_pub = rospy.Publisher('/reflex_takktile/command_motor_force', ForceCommand, queue_size=1)

    rospy.Subscriber('/reflex_takktile/hand_state', Hand, hand_state_cb)

    print "Hand initialized!!! Calibrate tactile"

    raw_input('start calibrating fingers:[Enter]')
    calibrate_fingers()
    raw_input('Done!')

    calibrate_tactile()
    rospy.sleep(0.5)

    print "Calibration Done!"

    f1 = 0.0
    f2 = 0.0
    f3 = 0.0
    reshape = 0.0

    f1_spd = 0.2
    f2_spd = 0.2
    f3_spd = 0.2
    reshape_spd = 0.1

    f1_limit = 2.8
    f2_limit = 2.8
    f3_limit = 3
    reshape_limit = 1.75

    control_f1_spd = 0
    control_f2_spd = 0
    control_f3_spd = 0
    control_reshape_spd = 0

    count = 0
    acc = 0.1
    try:
        print msg
        print vels(f1_spd, f2_spd, f3_spd)
        while(1):
            key = getKey()

            if key in moveBindings.keys():
                if moveBindings[key][0] != 0:
                    f1 = moveBindings[key][0]
                elif moveBindings[key][1] != 0:
                    f2 = moveBindings[key][1]
                elif moveBindings[key][2] != 0:
                    f3 = moveBindings[key][2]
                else:
                    reshape = moveBindings[key][3]
                print key
                count = 0
            elif key in speedBindings.keys():
                f1_spd = f1_spd * speedBindings[key][0]
                f2_spd = f2_spd * speedBindings[key][1]
                f3_spd = f3_spd * speedBindings[key][2]
                reshape_spd = 0.1
                count = 0
            elif key == 'f':
                force_pub.publish(ForceCommand(f1=200,f2=200,f3=300))
                rospy.sleep(10)

            elif key == ' ':

                vel_pub.publish(VelocityCommand(0,0,0,0))
                f1 = 0.0
                f2 = 0.0
                f3 = 0.0
                reshape = 0.0
                rospy.sleep(0.5)
                print "stop!"
                continue

            else:
                count = count + 1
                if count > 10:
                    f1 = 0
                    f2 = 0
                    f3 = 0
                if key == '\x03' or key == 'a':
                    break

            target_f1 = f1_spd * f1
            target_f2 = f2_spd * f2
            target_f3 = f3_spd * f3
            target_reshape = reshape_spd * reshape

            if hand_state.motor[0].joint_angle > f1_limit:
            	target_f1 = 0
            if hand_state.motor[1].joint_angle > f2_limit:
            	target_f2 = 0
            if hand_state.motor[2].joint_angle > f3_limit:
            	target_f3 = 0
            if hand_state.motor[2].joint_angle > reshape_limit:
            	target_reshape = 0


            if target_f1 > control_f1_spd:
                control_f1_spd = min( target_f1, control_f1_spd + acc )
            elif target_f1 < control_f1_spd:
                control_f1_spd = max( target_f1, control_f1_spd - acc )
            else:
                control_f1_spd = target_f1


            if target_f2 > control_f2_spd:
                control_f2_spd = min( target_f2, control_f2_spd + acc )
            elif target_f2 < control_f2_spd:
                control_f2_spd = max( target_f2, control_f2_spd - acc )
            else:
                control_f2_spd = target_f2

            if target_f3 > control_f3_spd:
                control_f3_spd = min( target_f3, control_f3_spd + acc )
            elif target_f3 < control_f3_spd:
                control_f3_spd = max( target_f3, control_f3_spd - acc )
            else:
                control_f3_spd = target_f3

            if target_f3 > control_f3_spd:
                control_f3_spd = min( target_f3, control_f3_spd + acc )
            elif target_f3 < control_f3_spd:
                control_f3_spd = max( target_f3, control_f3_spd - acc )
            else:
                control_f3_spd = target_f3

            if target_reshape > control_reshape_spd:
                control_reshape_spd = min( target_reshape, control_reshape_spd + acc )
            elif target_reshape < control_reshape_spd:
                control_reshape_spd = max( target_reshape, control_reshape_spd - acc )
            else:
                control_reshape_spd = target_reshape

            print msg
            print vels(control_f1_spd, control_f2_spd, control_f3_spd)
            print "(%f, %f, %f, %f)" %(f1_spd,f2_spd, f3_spd, reshape_spd)
            print hand_state.finger[0].pressure
            print hand_state.finger[1].pressure
            print hand_state.finger[2].pressure

            vel_pub.publish(VelocityCommand(control_f1_spd,control_f2_spd,control_f3_spd,target_reshape))
            rospy.sleep(0.2)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print e

    finally:
        pos_pub.publish(PoseCommand(0, 0, 0, 0))
        rospy.sleep(2)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

