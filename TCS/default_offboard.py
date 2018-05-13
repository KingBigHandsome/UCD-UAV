#!/usr/bin/env python
'''
Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

# import ROS libraries
import time
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import geometry_msgs
import nav_msgs.msg
import time
from datetime import datetime
from UAV_Task import *
import TCS_util

import thread
import math
import sys
import signal
import subprocess


# signal.SIGINT stop the thread
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# make an instance of the State.msg
UAV_state = mavros_msgs.msg.State() 

current_position = TCS_util.vector3()

precision = 0.1

takeoff_altitude = 2.0

emergency_sw = False

# Definitions of various callback functions
def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided

def _rc_in_callback(topic):
    """RCIN subscriber callback function Topic: /mavros/rc/in
    """
    global emergency_sw
    rc_in_channels = topic.channels

    if (rc_in_channels[5] <= 1750):
        emergency_sw = True
        #rospy.loginfo("emergency_sw is {}".format(emergency_sw))
    else:
        emergency_sw = False
        #rospy.loginfo("emergency_sw is {}".format(emergency_sw))

def local_position_cb(topic):
    """local position subscriber callback function Topic: /mavros/local_position/pose
    """
    current_position.is_init = True
    current_position.x = topic.pose.position.x
    current_position.y = topic.pose.position.y
    current_position.z = topic.pose.position.z

def is_reached():
    """Check if the UAV reached the takeoff altitude
    """
    if (abs(current_position.z-takeoff_altitude) < precision):
        print "Takeoff Finished!"
        return True
    else:
        return False


def main():

    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')

    # setup subscriber
    state_sub = rospy.Subscriber(mavros.get_topic('state'),mavros_msgs.msg.State, _state_callback)
    rc_in     = rospy.Subscriber(mavros.get_topic('rc','in'),mavros_msgs.msg.RCIn, _rc_in_callback)
    position_local_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                          geometry_msgs.msg.PoseStamped,
                                          local_position_cb)
    # setup services
    set_arming  = rospy.ServiceProxy('/mavros/cmd/arming'  , mavros_msgs.srv.CommandBool)
    set_mode    = rospy.ServiceProxy('/mavros/set_mode'    , mavros_msgs.srv.SetMode)
    set_home    = rospy.ServiceProxy('/mavros/cmd/set_home', mavros_msgs.srv.CommandHome)
    set_takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff' , mavros_msgs.srv.CommandTOL)

    # read task list
    Task_mgr = TCS_util.Task_manager('task_list.log')

    # start setpoint_update instance
    # In Firmware PX4, this function defined in Class update_setpoint is necessary to keep the flight mode in 'OFFBOARD',
    # But, it seems like unnecessary in Ardupilot.
    setpoint_keeper = TCS_util.update_setpoint(rospy)

    while(not UAV_state.connected):
        #rospy.loginfo("Waiting for vaild connection!")
        rate.sleep()

    while(UAV_state.mode != "GUIDED"):
        set_mode(0,'GUIDED')
        #rospy.loginfo("Please set mode to S'GUIDED'!")
        rate.sleep()

    while(not UAV_state.armed):
        set_arming(True)
        #rospy.loginfo("Please arm!")
        rate.sleep()
    # Set the current GPS position as home_position, otherwise, it is aviailable to set it by customized lat/lng/alt
    while(not set_home(True,0.0,0.0,0.0)):
        #rospy.loginfo("Please set_home!")
        rate.sleep()

    # Takeoff to specified altitude
    while(not set_takeoff(0.0,0.0,0.0,0.0,takeoff_altitude)):
        #rospy.loginfo("Please set_takeoff!")
        rate.sleep()

    while(not is_reached()):
        #rospy.loginfo("Waiting for takeoff to be done...")
        rate.sleep()

    #Delay for 5 seconds after takeoff, and then, start to execute tasks.
    time.sleep(10)

    last_request = rospy.Time.now()

    # enter the main loop
    while(True):

        if(emergency_sw):

            if( UAV_state.mode == "GUIDED" and UAV_state.armed is True):

                 #setpoint_keeper.update()

                if(Task_mgr.task_finished()):
                # If the current task has been done
                    rospy.loginfo("Current task is finished!")

                    if (not Task_mgr.alldone()):
                        Task_mgr.nexttask()
                    else:
                        # Current task has been done and no task left
                        rospy.loginfo("All tasks have been done! Flight mode changed to 'RTL'")
                        while (UAV_state.mode != "RTL"):
                            if((rospy.Time.now() - last_request) > rospy.Duration(2.0)):
                                set_mode(0,'RTL')
                                last_request = rospy.Time.now()
                            rate.sleep()
                        return 0
                rate.sleep()
        else:
            return 0
    #return 0

if __name__ == '__main__':
    main()
