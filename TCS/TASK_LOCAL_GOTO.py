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
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import TCS_util

# Declare some global variables.
# current position Position and Orientation
current_position = TCS_util.vector7()
# setpoint position
destination = TCS_util.vector3()

setpoint_position = TCS_util.vector3()

setpoint_raw_local  = TCS_util.vector4()

# precision setup. normally set it to 0.5m
precision = 0.5
pi = 3.1415926
frame_id='local_setpoint_raw'

def set_target(pose, x, y, z):
    """A wrapper assigning the x,y,z values
    to the pose. pose usually is type of
    geometry_msgs/PoseStamped.msg
    """
    pose.x = x
    pose.y = y
    pose.z = z

def update_msg(msg,vx,vy,vz,yaw):
    """A wrapper assigning the x,y,z values
    to the pose. pose usually is type of
    mavros_msgs/PositionTarget.msg
    """
    msg.header = mavros.setpoint.Header(frame_id=msg.header.frame_id,stamp=rospy.Time.now())
    msg.coordinate_frame = 1 # FRAME_LOCAL_NED = 1 Nort-Pitch East-Roll Down
    msg.type_mask = 967      # IGNORE_PX/PY/PZ IGNORE_AFX/AFY/AFZ FORCE 1+2+4+64+128+256+512
    msg.position.x = 0.0     # In this case,position.x/y/z were invalid by setting 'type_mask'
    msg.position.y = 0.0
    msg.position.z = 0.0
    msg.velocity.x = vx
    msg.velocity.y = vy
    msg.velocity.z = vz
    msg.yaw = yaw            # when yaw is 0, the drone towards to North
                             # the drone rotates clockwisely as the value increases.

def local_position_cb(topic):
    """local position subscriber callback function Topic: /mavros/local_position/pose
    """
    current_position.is_init = True
    current_position.px = topic.pose.position.x
    current_position.py = topic.pose.position.y
    current_position.pz = topic.pose.position.z
    current_position.ox = topic.pose.orientation.x
    current_position.oy = topic.pose.orientation.y
    current_position.oz = topic.pose.orientation.z
    current_position.ow = topic.pose.orientation.w
def is_reached(setpoint):
    """Check if the UAV reached the destination
    """

    if (abs(current_position.px-setpoint_position.x) < precision and
        abs(current_position.py-setpoint_position.y) < precision and
        abs(current_position.pz-setpoint_position.z) < precision):
        print "Point reached!"
        return True
    else:
        return False

def is_overtime(timestamp, overtime):
    """Check if the UAV reached the destination with in the set time
       if not, it will set mode to 'RTL'
    """
    if ((rospy.Time.now() - timestamp) > rospy.Duration(overtime)):
        print "Flight attempt over time!"
        return True
    else:
        return False

def main():

    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')

    theta = 0.0
    velocity_x = 0.0
    velocity_y = 0.0
    velocity_z = 0.0
    target_velocity = 2.0 # customized flight speed
    buffer_distance = 5.0 # the horizontal distance that the drone will slown down when it approaches the target position
    buffer_altitude = 1.0 # the Vertical   distance that the drone will slown down when it approaches the target position
    # setup setpoint_raw_local_pub
    setpoint_raw_local_pub = rospy.Publisher(mavros.get_topic('setpoint_raw', 'local'),
                                             mavros_msgs.msg.PositionTarget,
                                             queue_size=10)

    setpoint_msg = mavros.setpoint.PositionTarget(header=mavros.setpoint.Header(frame_id="local_setpoint_raw",
                                                                                stamp=rospy.Time.now()),)
    # setup service
    set_mode    = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

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
    destination.x=float(setpoint_arg[0])
    destination.y=float(setpoint_arg[1])
    destination.z=float(setpoint_arg[2])
    over_time = float(setpoint_arg[3])

    pre_flight = 'neutral'
    init_time = rospy.Time.now()
    # In this while loop, do the job.
    # Waiting for receive the first current position
    while(current_position.is_init is False):
        continue

    # When the current_position is available, calculate the current yaw value
    quaternion = (current_position.ox,current_position.oy,current_position.oz,current_position.ow)
    euler = euler_from_quaternion(quaternion,axes="sxyz")
    theta_yaw = euler[2] - pi/2.0

    # Firstly ascending to the target altitude and then flight to the destination
    if (destination.z - current_position.pz >2):
        pre_flight = 'ascending'

        # ascending
        set_target(setpoint_position,current_position.px,current_position.py,destination.z)

        while(not is_reached(setpoint_position)):

            m = current_position.px - setpoint_position.x  # Delat_x
            n = current_position.py - setpoint_position.y  # Delat_y
            p = current_position.pz - setpoint_position.z  # Delat_z
            q = math.sqrt(m * m + n * n)

            # during ascending, dynamically adjust its positon with a constant yaw
            if (not(m == 0 and n == 0)): # in case of denominator equal to zero

                if (m > 0 and n > 0): # The current position is in the upper-right conner of the target position

                    theta = math.asin(n/q)

                    velocity_x = -(0.05 * math.cos(theta))
                    velocity_y = -(0.05 * math.sin(theta))
                elif (m < 0 and n > 0): # upper-left conner

                    theta = math.asin(n/q)

                    velocity_x = (0.05 * math.cos(theta))
                    velocity_y = -(0.05 * math.sin(theta))
                elif (m < 0 and n < 0): # lower-left conner

                    theta = math.asin(-n/q)

                    velocity_x = (0.05 * math.cos(theta))
                    velocity_y = (0.05 * math.sin(theta))
                elif (m > 0 and n < 0): # lower-right conner

                    theta = math.asin(-n/q)

                    velocity_x = -(0.05 * math.cos(theta))
                    velocity_y =  (0.05 * math.sin(theta))
                else:
                    theta = theta

            setpoint_raw_local.vx = velocity_x
            setpoint_raw_local.vy = velocity_y
            # customized ascending speed
            if(p <= -buffer_altitude):
                setpoint_raw_local.vz = 0.5
            elif(abs(p) < buffer_altitude):
                setpoint_raw_local.vz = -0.5*p
            else:
                setpoint_raw_local.vz = -0.5
            setpoint_raw_local.yaw = theta_yaw

            update_msg(setpoint_msg,setpoint_raw_local.vx,setpoint_raw_local.vy,setpoint_raw_local.vz,setpoint_raw_local.yaw)

            setpoint_raw_local_pub.publish(setpoint_msg)

            task_watchdog.report_running()

            if (is_overtime(init_time, over_time)):
                set_mode(0,'RTL')
                break
            rate.sleep()

        setpoint_raw_local.vx = 0.0
        setpoint_raw_local.vy = 0.0
        setpoint_raw_local.vz = 0.0

        update_msg(setpoint_msg,setpoint_raw_local.vx,setpoint_raw_local.vy,setpoint_raw_local.vz,setpoint_raw_local.yaw)

        setpoint_raw_local_pub.publish(setpoint_msg)

        # flight to the target
        set_target(setpoint_position,destination.x,destination.y,destination.z)

        while(not is_reached(setpoint_position)):

            m = current_position.px - setpoint_position.x
            n = current_position.py - setpoint_position.y
            p = current_position.pz - setpoint_position.z
            q = math.sqrt(m * m + n * n)

            if (not(m == 0 and n == 0)):

                # slow down when distance to the target position within certain meters
                if (q < buffer_distance):
                    velocity = q/buffer_distance*target_velocity # the closer the distance, the lower the speed
                else:
                    velocity = target_velocity

                # automatically adjust altitude
                if (p >= 0.05):
                    velocity_z = -0.1
                elif (p<= -0.05):
                    velocity_z = 0.1
                else:
                    velocity_z = 0

                if (m > 0 and n > 0):
                    theta = math.asin(n/q)
                    theta_yaw = theta + pi/2.0
                    velocity_x = -(velocity * math.cos(theta))
                    velocity_y = -(velocity * math.sin(theta))
                elif (m < 0 and n > 0):
                    theta = math.asin(n/q)
                    theta_yaw = 1.5*pi - theta
                    velocity_x = (velocity * math.cos(theta))
                    velocity_y = -(velocity * math.sin(theta))
                elif (m < 0 and n < 0):
                    theta = math.asin(-n/q)
                    theta_yaw = theta - pi/2.0
                    velocity_x = (velocity * math.cos(theta))
                    velocity_y = (velocity * math.sin(theta))
                elif (m > 0 and n < 0):
                    theta = math.asin(-n/q)
                    theta_yaw = pi/2.0 -theta
                    velocity_x = -(velocity * math.cos(theta))
                    velocity_y =  (velocity * math.sin(theta))
                else:
                    theta = theta

            setpoint_raw_local.vx = velocity_x
            setpoint_raw_local.vy = velocity_y
            setpoint_raw_local.vz = velocity_z
            setpoint_raw_local.yaw = theta_yaw

            update_msg(setpoint_msg,setpoint_raw_local.vx,setpoint_raw_local.vy,setpoint_raw_local.vz,setpoint_raw_local.yaw)

            setpoint_raw_local_pub.publish(setpoint_msg)

            task_watchdog.report_running()

            if (is_overtime(init_time, over_time)):
                set_mode(0,'RTL')
                break
            rate.sleep()

    # Firstly flight to the destination holding the current altitude and the descend to the target altitude
    elif (current_position.pz - destination.z >2):
        pre_flight = 'descending'
        # flight to the target first
        set_target(setpoint_position,destination.x,destination.y,current_position.pz)

        while(not is_reached(setpoint_position)):

            m = current_position.px - setpoint_position.x
            n = current_position.py - setpoint_position.y
            p = current_position.pz - setpoint_position.z
            q = math.sqrt(m * m + n * n)

            if (not(m == 0 and n == 0)):

                # slow down when distance to the target position within certain meters
                if (q < 5.0):
                    velocity = q/5.0*target_velocity
                else:
                    velocity = target_velocity

                 # automatically adjust altitude
                if (p >= 0.05):
                    velocity_z = -0.1
                elif (p<= -0.05):
                    velocity_z = 0.1
                else:
                    velocity_z = 0

                if (m > 0 and n > 0):
                    theta = math.asin(n/q)
                    theta_yaw = theta + pi/2.0
                    velocity_x = -(velocity * math.cos(theta))
                    velocity_y = -(velocity * math.sin(theta))
                elif (m < 0 and n > 0):
                    theta = math.asin(n/q)
                    theta_yaw = 1.5*pi - theta
                    velocity_x = (velocity * math.cos(theta))
                    velocity_y = -(velocity * math.sin(theta))
                elif (m < 0 and n < 0):
                    theta = math.asin(-n/q)
                    theta_yaw = theta - pi/2.0
                    velocity_x = (velocity * math.cos(theta))
                    velocity_y = (velocity * math.sin(theta))
                elif (m > 0 and n < 0):
                    theta = math.asin(-n/q)
                    theta_yaw = pi/2.0 -theta
                    velocity_x = -(velocity * math.cos(theta))
                    velocity_y =  (velocity * math.sin(theta))
                else:
                    theta = theta

            setpoint_raw_local.vx = velocity_x
            setpoint_raw_local.vy = velocity_y
            setpoint_raw_local.vz = velocity_z
            setpoint_raw_local.yaw = theta_yaw


            update_msg(setpoint_msg,setpoint_raw_local.vx,setpoint_raw_local.vy,setpoint_raw_local.vz,setpoint_raw_local.yaw)

            setpoint_raw_local_pub.publish(setpoint_msg)

            task_watchdog.report_running()

            if (is_overtime(init_time, over_time)):
                set_mode(0,'RTL')
                break
            rate.sleep()

        setpoint_raw_local.vx = 0.0
        setpoint_raw_local.vy = 0.0
        setpoint_raw_local.vz = 0.0

        update_msg(setpoint_msg,setpoint_raw_local.vx,setpoint_raw_local.vy,setpoint_raw_local.vz,setpoint_raw_local.yaw)

        setpoint_raw_local_pub.publish(setpoint_msg)

        # then descending
        set_target(setpoint_position,destination.x,destination.y,destination.z)

        while(not is_reached(setpoint_position)):

            m = current_position.px - setpoint_position.x  # Delat_x
            n = current_position.py - setpoint_position.y  # Delat_y
            p = current_position.pz - setpoint_position.z  # Delat_z
            q = math.sqrt(m * m + n * n)

            if (not(m == 0 and n == 0)): # in case of denominator equal to zero

                if (m > 0 and n > 0): # The current position is in the upper-right conner of the target position

                    theta = math.asin(n/q)

                    velocity_x = -(0.05 * math.cos(theta))
                    velocity_y = -(0.05 * math.sin(theta))
                elif (m < 0 and n > 0): # upper-left conner

                    theta = math.asin(n/q)

                    velocity_x = (0.05 * math.cos(theta))
                    velocity_y = -(0.05 * math.sin(theta))
                elif (m < 0 and n < 0): # lower-left conner

                    theta = math.asin(-n/q)

                    velocity_x = (0.05 * math.cos(theta))
                    velocity_y = (0.05 * math.sin(theta))
                elif (m > 0 and n < 0): # lower-right conner

                    theta = math.asin(-n/q)

                    velocity_x = -(0.05 * math.cos(theta))
                    velocity_y =  (0.05 * math.sin(theta))
                else:
                    theta = theta

            setpoint_raw_local.vx = velocity_x
            setpoint_raw_local.vy = velocity_y
            # customized ascending speed
            if(p >= buffer_altitude):
                setpoint_raw_local.vz = -0.5
            elif(abs(p) < buffer_altitude):
                setpoint_raw_local.vz = -0.5*p
            else:
                setpoint_raw_local.vz = 0.5
            setpoint_raw_local.yaw = theta_yaw

            update_msg(setpoint_msg,setpoint_raw_local.vx,setpoint_raw_local.vy,setpoint_raw_local.vz,setpoint_raw_local.yaw)

            setpoint_raw_local_pub.publish(setpoint_msg)

            task_watchdog.report_running()

            if (is_overtime(init_time, over_time)):
                set_mode(0,'RTL')
                break
            rate.sleep()

    # Flight to destination directly.
    else:

        set_target(setpoint_position,destination.x,destination.y,destination.z)

        while(not is_reached(setpoint_position)):

            m = current_position.px - setpoint_position.x
            n = current_position.py - setpoint_position.y
            p = current_position.pz - setpoint_position.z
            q = math.sqrt(m * m + n * n)

            if (not(m == 0 and n == 0)):

                # slow down when distance to the target position within certain meters
                if (q < 5.0):
                    velocity = q/5.0*target_velocity
                else:
                    velocity = target_velocity

                 # automatically adjust altitude
                if (p >= 0.05):
                    velocity_z = -0.1
                elif (p<= -0.05):
                    velocity_z = 0.1
                else:
                    velocity_z = 0

                if (m > 0 and n > 0):
                    theta = math.asin(n/q)
                    theta_yaw = theta + pi/2.0
                    velocity_x = -(velocity * math.cos(theta))
                    velocity_y = -(velocity * math.sin(theta))
                elif (m < 0 and n > 0):
                    theta = math.asin(n/q)
                    theta_yaw = 1.5*pi - theta
                    velocity_x = (velocity * math.cos(theta))
                    velocity_y = -(velocity * math.sin(theta))
                elif (m < 0 and n < 0):
                    theta = math.asin(-n/q)
                    theta_yaw = theta - pi/2.0
                    velocity_x = (velocity * math.cos(theta))
                    velocity_y = (velocity * math.sin(theta))
                elif (m > 0 and n < 0):
                    theta = math.asin(-n/q)
                    theta_yaw = pi/2.0 -theta
                    velocity_x = -(velocity * math.cos(theta))
                    velocity_y =  (velocity * math.sin(theta))
                else:
                    theta = theta

            setpoint_raw_local.vx = velocity_x
            setpoint_raw_local.vy = velocity_y
            setpoint_raw_local.vz = velocity_z
            setpoint_raw_local.yaw = theta_yaw

            update_msg(setpoint_msg,setpoint_raw_local.vx,setpoint_raw_local.vy,setpoint_raw_local.vz,setpoint_raw_local.yaw)

            setpoint_raw_local_pub.publish(setpoint_msg)

            task_watchdog.report_running()

            if (is_overtime(init_time, over_time)):
                set_mode(0,'RTL')
                break
            rate.sleep()

    # Arrived at the destination successfully, holding its position
    setpoint_raw_local.vx = 0.0
    setpoint_raw_local.vy = 0.0
    setpoint_raw_local.vz = 0.0
    setpoint_raw_local.yaw = theta_yaw

    update_msg(setpoint_msg,setpoint_raw_local.vx,setpoint_raw_local.vy,setpoint_raw_local.vz,setpoint_raw_local.yaw)

    setpoint_raw_local_pub.publish(setpoint_msg)

    #Publish the task status as FINISH
    task_watchdog.report_finish()

    return 0

if __name__ == '__main__':
    main()

