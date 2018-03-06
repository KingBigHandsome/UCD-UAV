#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''
'''
This is the TASK_LOCAL_GOTO
This task will guide the drone to the destination point 
by keeping sending setpoint_local
This task is able to check is_reached
once the UAV reached the point, the task will exit
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
import sensor_msgs.msg
import time
from datetime import datetime
from math import cos, sin

import TCS_util

# Declare some global variables.
GPS_TO_METER = 111170    #at 47degree latitude
m_per_deg_lat = 111006      #at 38.55 latitude
m_per_deg_lon = 87172  # at 38.55 latitude
# current position
current_local_position = TCS_util.vector3()
# setpoint message. The type will be changed later in main()
setpoint_msg = 0
# setpoint position
setpoint_position = TCS_util.vector3()
# global target
setpoint_GPS_target = TCS_util.vector3()
# global current
current_GPS_position = TCS_util.vector3()

raw_setpoint_position = TCS_util.vector3()
# precision setup. normally set it to 0.5m
precision = 0.5
GPS_precision = 0.0000005
# setup frame_id
frame_id='HYBRID_GOTO'

def set_target(pose, x, y, z):
    """A wrapper assigning the x,y,z values
    to the pose. pose usually is type of 
    mavros.setpoint.PoseStamped
    """
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.header=mavros.setpoint.Header(frame_id="hybrid_pose",stamp=rospy.Time.now())

def update_msg(msg):
    msg.header = mavros.setpoint.Header(frame_id=msg.header.frame_id,stamp=rospy.Time.now())

def local_position_cb(topic):
    """local position subscriber callback function Topic: /mavros/local_position/pose
    """
    current_local_position.is_init = True
    current_local_position.x = topic.pose.position.x
    current_local_position.y = topic.pose.position.y
    current_local_position.z = topic.pose.position.z

def GPS_position_cb(topic):
    """Global position subscriber callback function Topic: /mavros/global_position/global
    """
    current_GPS_position.is_init = True
    current_GPS_position.x = topic.latitude
    current_GPS_position.y = topic.longitude
    current_GPS_position.z = topic.altitude
    # print 'Now the GPS cb: {},{},{}\n'.format(current_GPS_position.x,current_GPS_position.y, current_GPS_position.z)

def convert_GPS_to_local(GPS_target, GPS_current, local_current, pose, altitude):
    latMid = (GPS_target.x-GPS_current.x)/2
    m_per_deg_lat = 111132.954 - 559.822 * cos( 2.0 * latMid ) + 1.175 * cos( 4.0 * latMid);
    m_per_deg_lon = (3.14159265359/180 ) * 6367449 * cos ( latMid );
    pose.pose.position.x = local_current.x+m_per_deg_lat*(GPS_target.x-GPS_current.x)
    pose.pose.position.y = local_current.y+m_per_deg_lon*(GPS_target.x-GPS_current.x)
    pose.pose.position.z = altitude
    pose.header=mavros.setpoint.Header(frame_id="hybrid_pose",stamp=rospy.Time.now())
    # print "Target GPS.x: {}, current GPS.x: {}, difference: {}\n".format(GPS_target.x, GPS_current.x, GPS_target.x-GPS_current.x)


def is_reached(setpoint):
    """Check if the UAV reached the destination
    """
    x = setpoint.pose.position.x
    y = setpoint.pose.position.y
    z = setpoint.pose.position.z
    if (abs(current_local_position.x-x) < precision and
            abs(current_local_position.y-y) < precision and
            abs(current_local_position.z-z) < precision):
        print "Point reached!"
        return True
    else:
        return False

def is_GPS_reached(setpoint):
    """Check if the UAV reached the destination
    """
    x = setpoint.x
    y = setpoint.y
    if (abs(current_GPS_position.x-x) < GPS_precision and
            abs(current_GPS_position.y-y) < GPS_precision):
        print "GPS Point reached!"
        return True
    else:
        print 'current_GPS_position.x: {}, setpoint.x: {} , difference: {}'.format(current_GPS_position.x, x, abs(current_GPS_position.x-x))
        return False

def is_overtime(timestamp, overtime):
    if ((rospy.Time.now() - timestamp) > rospy.Duration(overtime)):
        print "Flight attempt over time!"
        return True
    else:
        return False
   

def main():
    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')
    
    # setup local pub
    setpoint_local_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), 
    									 geometry_msgs.msg.PoseStamped, 
    									 queue_size=10)

    # setup setpoint_msg
    setpoint_msg = mavros.setpoint.PoseStamped(header=mavros.setpoint.Header(frame_id="hybrid_pose",
               																 stamp=rospy.Time.now()),)

    # setup local sub
    position_local_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
    									  geometry_msgs.msg.PoseStamped, 
    									  local_position_cb)
    # setup GPS sub
    position_GPS_sub   = rospy.Subscriber(mavros.get_topic('global_position', 'global'),
        								  sensor_msgs.msg.NavSatFix, 
        								  GPS_position_cb)

    # setup task pub
    task_watchdog = TCS_util.Task_watchdog('TASK_HYBRID_GOTO')

    # interprete the input position
    setpoint_arg = sys.argv[1].split(' ')
    raw_setpoint_position.x=float(setpoint_arg[0])
    raw_setpoint_position.y=float(setpoint_arg[1])
    raw_setpoint_position.z=float(setpoint_arg[2])
    setpoint_GPS_target.x = raw_setpoint_position.x
    setpoint_GPS_target.y = raw_setpoint_position.y
    setpoint_GPS_target.z = raw_setpoint_position.z
    over_time = float(setpoint_arg[3])
    print "X: {}, Y: {}, Z: {}".format(raw_setpoint_position.x,
    								   raw_setpoint_position.y, 
    								   raw_setpoint_position.z)

    pre_flight = 'neutral'
    init_time = rospy.Time.now()
    # In this while loop, do the job.
    # Waiting for receive the first current position
    while(current_local_position.is_init is False or current_GPS_position.is_init is False):
        continue
    if (raw_setpoint_position.z - current_local_position.z >2):
        pre_flight = 'ascending'
        # ascending
        set_target(setpoint_msg,
            current_local_position.x,
            current_local_position.y,
            raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            update_msg(setpoint_msg)
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()
        # flight to the target
        convert_GPS_to_local(setpoint_GPS_target, current_GPS_position, current_local_position,
        					 setpoint_msg,raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            convert_GPS_to_local(setpoint_GPS_target, current_GPS_position, current_local_position,
                                 setpoint_msg,raw_setpoint_position.z)
            rate.sleep()

    elif (current_local_position.z - raw_setpoint_position.z >2):
        pre_flight = 'descending'
        # flight to the target first
        convert_GPS_to_local(setpoint_GPS_target, current_GPS_position, current_local_position,
        					 setpoint_msg,raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            convert_GPS_to_local(setpoint_GPS_target, current_GPS_position, current_local_position,
                				 setpoint_msg,raw_setpoint_position.z)
            rate.sleep()
        # descending
        set_target(setpoint_msg,
            raw_setpoint_position.x,
            raw_setpoint_position.y,
            raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            update_msg(setpoint_msg)
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()


    else:
        convert_GPS_to_local(setpoint_GPS_target, current_GPS_position, current_local_position,
        					 setpoint_msg,raw_setpoint_position.z)
        while(not is_reached(setpoint_msg)):
            setpoint_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            convert_GPS_to_local(setpoint_GPS_target, current_GPS_position, current_local_position,
                				 setpoint_msg,raw_setpoint_position.z)
            rate.sleep()
    
    # TODO: publish the task status as FINISH
    task_watchdog.report_finish()

    return 0


if __name__ == '__main__':
    main()

