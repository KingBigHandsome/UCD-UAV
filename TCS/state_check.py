#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import geometry_msgs
import time
from datetime import datetime
from UAV_Task import *
import TCS_util
# import utilities
import thread
import math
import sys
import signal
import subprocess

from tf.transformations import quaternion_from_euler
target_position_x = 5
target_position_y = 5

emergency_sw = 'Neutral'
yaw = math.atan(target_position_y/target_position_x)
# signal.SIGINT stop the thread
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

UAV_state = mavros_msgs.msg.State() 
rc_out_channels = []

# Definitions of various callback functions
def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided
def _rc_out_callback(topic):

    rc_out_channels = topic.channels
   
    global emergency_sw
    
    rospy.loginfo("The output of switch B is {}".format(rc_out_channels[5]))        
    #print "The output of switch B is ", rc_out_channels[5]
    
    if (rc_out_channels[5] <= 1250):
        emergency_sw = 'Up'
        rospy.loginfo("emergency_sw is {}".format(emergency_sw))
    elif (rc_out_channels[5] > 1250 and rc_out_channels[5] < 1750):
        emergency_sw = 'Neutral'
        rospy.loginfo("emergency_sw is {}".format(emergency_sw))
    else:
        emergency_sw = 'Down'
        rospy.loginfo("emergency_sw is {}".format(emergency_sw))




def main():
    rospy.init_node('state_check', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')
    
    #global emergency_s
    
    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),mavros_msgs.msg.State, _state_callback)
    rc_out    = rospy.Subscriber(mavros.get_topic('rc','out'),mavros_msgs.msg.RCOut, _rc_out_callback)

    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    set_mode   = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

    while(not UAV_state.connected):
        rate.sleep()
    #set_mode(0,'STABILIZE')
    #set_arming(True)
    rospy.loginfo("Pre start finished!")

    q = quaternion_from_euler(10, 10, yaw)
    #print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
    print "The quaternion representation is %s." % (math.asin(0.866025))

    last_request = rospy.Time.now()
    while(False):
        if( UAV_state.mode != "GUIDED" and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            if( set_mode(0,'GUIDED')):
                rospy.loginfo("'GUIDED' mode enabled")
            last_request = rospy.Time.now()
        rospy.loginfo("The current flight mode is: {}".format(UAV_state.mode))
        rospy.loginfo("The current arm state is: {}".format(UAV_state.armed))
        rospy.loginfo("emergency_sw is {}".format(emergency_sw))
        if(emergency_sw != 'Down'):
            if(emergency_sw == 'Neutral'):
                rospy.loginfo("Neutral")
            else:
                rospy.loginfo("Up")
        else:
            rospy.loginfo("Down")
        rate.sleep()

if __name__ == '__main__':
    main()
