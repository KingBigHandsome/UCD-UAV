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
import nav_msgs.msg
import time
from datetime import datetime
from tf.transformations import quaternion_from_euler
import TCS_util

# Declare some global variables.
# current position
current_position = TCS_util.vector3()
# setpoint message. The type will be changed later in main()
setpoint_msg = 0
# setpoint position

raw_setpoint_position = TCS_util.vector3()

setpoint_position = TCS_util.vector3()

set_velocity_raw  = TCS_util.vector4()

# precision setup. normally set it to 0.5m
precision = 0.5
# setup frame_id
frame_id='local_setpoint_raw'

def set_target(setpoint_position, x, y, z):
    """A wrapper assigning the x,y,z values
    to the pose. pose usually is type of
    mavros.setpoint.PoseStamped
    """
    setpoint_position.x = x
    setpoint_position.y = y
    setpoint_position.z = z

def update_msg(msg,vx,vy,vz,yaw):
    msg.header = mavros.setpoint.Header(frame_id=msg.header.frame_id,stamp=rospy.Time.now())
    msg.coordinate_frame = 1
    msg.type_mask = 967
    msg.position.x = 0.0
    msg.position.y = 0.0
    msg.position.z = 0.0
    msg.velocity.x = vx
    msg.velocity.y = vy
    msg.velocity.z = vz
    msg.yaw = yaw

def local_position_cb(topic):
    """local position subscriber callback function Topic: /mavros/local_position/pose
    """
    current_position.is_init = True
    current_position.x = topic.pose.position.x
    current_position.y = topic.pose.position.y
    current_position.z = topic.pose.position.z

def is_reached(setpoint):
    """Check if the UAV reached the destination
    """

    if (abs(current_position.x-setpoint_position.x) < precision and
        abs(current_position.y-setpoint_position.y) < precision and
        abs(current_position.z-setpoint_position.z) < precision):
        print "Point reached!"
        return True
    else:
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

    # setup setpoint_raw_local_pub
    setpoint_raw_local_pub = rospy.Publisher(mavros.get_topic('setpoint_raw', 'local'),
                                             mavros_msgs.msg.PositionTarget,
                                             queue_size=10)

    setpoint_msg = mavros_msgs.msg.PositionTarget
    #setpoint_msg.header.stamp = rospy.Time.now()
    #setpoint_msg.header.frame_id = "local_setpoint_raw"



    # setup setpoint_msg
    #setpoint_msg = mavros.setpoint.PoseStamped(header=mavros.setpoint.Header(frame_id="local_pose",stamp=rospy.Time.now()),)

    # setup position_local_sub
    position_local_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                          geometry_msgs.msg.PoseStamped,
                                          local_position_cb)

    # make a instance of the class task_watcdog declared in the file TCS.util
    task_watchdog = TCS_util.Task_watchdog('TASK_LOCAL_GOTO')

    # interpret the input position
    # sys.argv is a list in Python which contains the command-line arguments passed to the script.
    # sys.argv[0] : This is the name of the script
    # len(sys.argv) : Number of arguments
    # str(sys.argv) : The arguments are ['name of the script','arg1','arg2'...]
    # spilt into list use " "e.g.setpoint[0]=5 setpoint[1]=0 setpoint[2]=5 setpoint[3]=20
    setpoint_arg = sys.argv[1].split(' ')
    raw_setpoint_position.x=float(setpoint_arg[0])
    raw_setpoint_position.y=float(setpoint_arg[1])
    raw_setpoint_position.z=float(setpoint_arg[2])
    over_time = float(setpoint_arg[3])
    print "The Destination is : X = {}, y = {}, Z = {}".format(raw_setpoint_position.x,
                                                               raw_setpoint_position.y,
                                                               raw_setpoint_position.z)

    pre_flight = 'neutral'
    init_time = rospy.Time.now()
    # In this while loop, do the job.
    # Waiting for receive the first current position
    while(current_position.is_init is False):
        continue
    if (raw_setpoint_position.z - current_position.z >2):
        pre_flight = 'ascending'
        # ascending
        set_target(setpoint_position,
                   current_position.x,
                   current_position.y,
                   raw_setpoint_position.z)
        while(not is_reached(setpoint_position)):
            set_velocity_raw.vx =0.0
            set_velocity_raw.vy = 0.0
            set_velocity_raw.vz = 0.2
            set_velocity_raw.yaw = 0.0
            update_msg(setpoint_msg,set_velocity_raw.vx,set_velocity_raw.vy,set_velocity_raw.vz,set_velocity_raw.yaw)
            setpoint_raw_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()
        set_velocity_raw.vz = 0.0
        update_msg(setpoint_msg,set_velocity_raw.vx,set_velocity_raw.vy,set_velocity_raw.vz,set_velocity_raw.yaw)
        setpoint_raw_local_pub.publish(setpoint_msg)
        # flight to the target
        set_target(setpoint_position,
                   raw_setpoint_position.x,
                   raw_setpoint_position.y,
                   raw_setpoint_position.z)
        while(not is_reached(setpoint_position)):
            if(raw_setpoint_position.x != current_position.x):
                theta = math.atan((raw_setpoint_position.y-current_position.y)/(raw_setpoint_position.x-current_position.x))
                degree = thera/3.1415926*180
            set_velocity_raw.vx = 5*sin(degree)
            set_velocity_raw.vy = 5*cos(degree)
            set_velocity_raw.vz = 0.0
            set_velocity_raw.yaw = theta
            update_msg(setpoint_msg,set_velocity_raw.vx,set_velocity_raw.vy,set_velocity_raw.vz,set_velocity_raw.yaw)
            setpoint_raw_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()

    elif (current_position.z - raw_setpoint_position.z >2):
        pre_flight = 'descending'
        # flight to the target first
        set_target(setpoint_position,
                   raw_setpoint_position.x,
                   raw_setpoint_position.y,
                   current_position.z)
        while(not is_reached(setpoint_position)):
            if(raw_setpoint_position.x != current_position.x):
                theta = math.atan((raw_setpoint_position.y-current_position.y)/(raw_setpoint_position.x-current_position.x))
                degree = thera/3.1415926*180
            set_velocity_raw.vx = 5*sin(degree)
            set_velocity_raw.vy = 5*cos(degree)
            set_velocity_raw.vz = 0.0
            set_velocity_raw.yaw = theta
            update_msg(setpoint_msg,set_velocity_raw.vx,set_velocity_raw.vy,set_velocity_raw.vz,set_velocity_raw.yaw)
            setpoint_raw_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()
        set_velocity_raw.vx = 0.0
        set_velocity_raw.vy = 0.0
        set_velocity_raw.vz = 0.0
        update_msg(setpoint_msg,set_velocity_raw.vx,set_velocity_raw.vy,set_velocity_raw.vz,set_velocity_raw.yaw)
        # descending
        set_target(setpoint_position,
                   raw_setpoint_position.x,
                   raw_setpoint_position.y,
                   raw_setpoint_position.z)
        while(not is_reached(setpoint_position)):
            set_velocity_raw.vx =0.0
            set_velocity_raw.vy = 0.0
            set_velocity_raw.vz = -0.2
            set_velocity_raw.yaw = 0.0
            update_msg(setpoint_msg,set_velocity_raw.vx,set_velocity_raw.vy,set_velocity_raw.vz,set_velocity_raw.yaw)
            setpoint_raw_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()


    else:
        set_target(setpoint_position,
                   raw_setpoint_position.x,
                   raw_setpoint_position.y,
                   raw_setpoint_position.z)
        while(not is_reached(setpoint_position)):
            if(raw_setpoint_position.x != current_position.x):
                theta = math.atan((raw_setpoint_position.y-current_position.y)/(raw_setpoint_position.x-current_position.x))
                degree = thera/3.1415926*180
            set_velocity_raw.vx = 5*sin(degree)
            set_velocity_raw.vy = 5*cos(degree)
            set_velocity_raw.vz = 0.0
            set_velocity_raw.yaw = theta
            update_msg(setpoint_msg,set_velocity_raw.vx,set_velocity_raw.vy,set_velocity_raw.vz,set_velocity_raw.yaw)
            setpoint_raw_local_pub.publish(setpoint_msg)
            task_watchdog.report_running()
            if (is_overtime(init_time, over_time)):
                break
            rate.sleep()
    set_velocity_raw.vx = 0.0
    set_velocity_raw.vy = 0.0
    set_velocity_raw.vz = 0.0
    update_msg(setpoint_msg,set_velocity_raw.vx,set_velocity_raw.vy,set_velocity_raw.vz,set_velocity_raw.yaw)

    #Publish the task status as FINISH
    task_watchdog.report_finish()

    return 0


if __name__ == '__main__':
    main()

