#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''
'''
This is the TASK_SIMPLE_HOVER
This task will hover at the current position until the
time delay reached
TODO:
implement an appropriate platfrom recognition
implement task publisher that talking to main TCS process
that this TASK has finished its job.
'''

# import utilities
import math
import sys
import signal
import subprocess
import os
import platform
if (platform.uname()[1]=='ubuntu'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname()[1]=='edison'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname()[1]=='xiaoguang'):
    # this is workstation in 079 Lab
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import geometry_msgs
import nav_msgs.msg
import time
from datetime import datetime

import TCS_util



# Declare some global variables.
# current position
current_position = TCS_util.vector3()
# setpoint message. The type will be changed later in main()
setpoint_msg = 0
# setpoint position
setpoint_position = TCS_util.vector3()
# precision setup. normally set it to 0.5m
precision = 0.5
# setup frame_id
frame_id='SIMPLE_HOVER'

def set_target(pose, x, y, z):
    """A wrapper assigning the x,y,z values
    to the pose. pose usually is type of
    mavros.setpoint.PoseStamped
    """
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0

    pose.header=mavros.setpoint.Header(frame_id="SIMPLE_HOVER",stamp=rospy.Time.now())

def local_position_cb(topic):
    """local position subscriber callback function
    """
    if (current_position.is_init is False):
        current_position.x = topic.pose.position.x
        current_position.y = topic.pose.position.y
        current_position.z = topic.pose.position.z
        print "Current position set to: %.2f %.2f %.2f"%(current_position.x,
                                                         current_position.y,
                                                         current_position.z)
        current_position.is_init=True

def main():
    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')
    # setup local pub /mavros/sepoint_position/local
    setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

    # setup setpoint_msg
    setpoint_msg = mavros.setpoint.PoseStamped(header=mavros.setpoint.Header(frame_id="SIMPLE_HOVER",
                                                                             stamp=rospy.Time.now()),)

    # setup local sub
    position_local_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                                            geometry_msgs.msg.PoseStamped,
                                                            local_position_cb)

    # setup task pub
    task_watchdog = TCS_util.Task_watchdog('TASK_SIMPLE_HOVER')

    # interprete the input position
    setpoint_arg = sys.argv[1].split(' ')
    delay_set=float(setpoint_arg[0])
    print "The UAV will hover for %.2f seconds."% delay_set
    time_stamp_start=rospy.Time.now()

    # In this while loop, do the job.
    while(not (rospy.Time.now()-time_stamp_start>rospy.Duration(delay_set))):
        # setup setpoint poisiton and prepare to publish the position
        if(current_position.is_init is True):
            set_target(setpoint_msg,
                           current_position.x,
                       current_position.y,
                       current_position.z)
        # publish the current position letting UAV hover that the position
        setpoint_local_pub.publish(setpoint_msg)
        # Publish the task status as RUNNING
        task_watchdog.report_running()

        rate.sleep()

    # Publish the task status as FINISH
    task_watchdog.report_finish()

    return 0


if __name__ == '__main__':
    main()

