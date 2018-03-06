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


# signal.SIGINT stop the thread
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


#current_pose = vector3()
# make an instance of the State.msg stored in the folder of mavros_msgs/msg/State.msg
UAV_state = mavros_msgs.msg.State() 


# Definitions of various callback functions
def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided
    
#uncalled function
def _setpoint_position_callback(topic):
    pass
    
#uncalled function
def _set_pose(pose, x, y, z):
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.header=mavros.setpoint.Header(frame_id="att_pose",stamp=rospy.Time.now())
    
#uncalled function
def is_reached(current, setpoint):
    if (abs(current.x-setpoint.pose.position.x) < 0.5 and
        abs(current.y-setpoint.pose.position.y) < 0.5 and
        abs(current.z-setpoint.pose.position.z) < 0.5):
        print "Point reached!"
        return True
    else:
        return False
        
#uncalled function
def update_setpoint():
    pass

def main():
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')

    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),mavros_msgs.msg.State, _state_callback)
    # /mavros/local_position/pose
    # local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
    #     									SP.PoseStamped, 
    #										_local_position_callback)
    # /mavros/setpoint_raw/target_local
    #setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
    #    								  mavros_msgs.msg.PositionTarget,
    #    								  _setpoint_position_callback)

    # setup publisher
    # /mavros/setpoint_position/local
    
    setpoint_local_pub  = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), 
        								  geometry_msgs.msg.PoseStamped, 
        								  queue_size=10)
    setpoint_global_pub = rospy.Publisher(mavros.get_topic('setpoint_raw', 'global'),
        								  mavros_msgs.msg.GlobalPositionTarget, 
        								  queue_size=10)
    # setup service
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
    set_mode   = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  

    setpoint_local_msg = mavros.setpoint.PoseStamped(header=mavros.setpoint.Header(
    										   		 frame_id="setpoint_local",
                							   		 stamp=rospy.Time.now()),)

    setpoint_global_msg = mavros_msgs.msg.GlobalPositionTarget(header=mavros.setpoint.Header(
                											   frame_id="setpoint_gps",
               	 											   stamp=rospy.Time.now()),)

    #read task list
    Task_mgr = TCS_util.Task_manager('task_list.log')

    # start setpoint_update instance
    setpoint_keeper = TCS_util.update_setpoint(rospy)

    # wait for FCU connection
    while(not UAV_state.connected):
        rate.sleep()

    # initialize the setpoint
    setpoint_local_msg.pose.position.x = 0
    setpoint_local_msg.pose.position.y = 0
    setpoint_local_msg.pose.position.z = 3

    setpoint_global_msg.coordinate_frame = 11
    setpoint_global_msg.type_mask = 8+16+32+128+256
    setpoint_global_msg.latitude = 47.3978800
    setpoint_global_msg.longitude = 8.5455920
    setpoint_global_msg.altitude = 10
    setpoint_global_msg.yaw = 90
    setpoint_global_msg.yaw_rate =10

    mavros.command.arming(True)

    # send 100 setpoints before starting               
    for i in range(0,50):
        #setpoint_local_pub.publish(setpoint_local_msg)
        setpoint_global_pub.publish(setpoint_global_msg)
        rate.sleep()

    set_mode(0,'OFFBOARD')
    print("Pre start finished!")

    last_request = rospy.Time.now()

    # enter the main loop
    while(True):
        #rospy.loginfo("The current flight mode is: {};The current arm state is: {}".format(UAV_state.mode,UAV_state.armed))
        rospy.loginfo("The current flight mode is: {}".format(UAV_state.mode))
        rospy.loginfo("The current arm state is: {}".format(UAV_state.armed))
        if( UAV_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            if( set_mode(0,'OFFBOARD')):
                print "Offboard enabled"
            last_request = rospy.Time.now()
        else:
            if(not UAV_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if( mavros.command.arming(True)):
                    print "Vehicle armed"
                last_request = rospy.Time.now()

        # update setpoint to stay in offboard mode
        # TCS.util.py: class update_setpoint: Function:update
        
        setpoint_keeper.update()
        
        if(Task_mgr.task_finished()):
            # If the current task has been done
            rospy.loginfo("Current task is finished!")
            if (not Task_mgr.alldone()):
                # If there are tasks left
                Task_mgr.nexttask()

            else:
                # Current task has been done and no task left
                rospy.loginfo("All tasks have been done!")
                while (UAV_state.mode != "AUTO.LAND"):
                    set_mode(0,'AUTO.LAND')
                    rate.sleep()
                return 0

        rate.sleep()
    return 0

if __name__ == '__main__':
    main()
