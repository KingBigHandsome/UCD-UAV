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

UAV_state = mavros_msgs.msg.State() 

# Definitions of various callback functions
def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided

def main():
    rospy.init_node('state_check', anonymous=True)
    rate = rospy.Rate(10)
    mavros.set_namespace('/mavros')

    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),mavros_msgs.msg.State, _state_callback)

    while(not UAV_state.connected):
        rate.sleep()
    while(True):
 
        rospy.loginfo("The current flight mode is: {}".format(UAV_state.mode))
        rospy.loginfo("The current arm state is: {}".format(UAV_state.armed))
       
        rate.sleep()

if __name__ == '__main__':
    main()