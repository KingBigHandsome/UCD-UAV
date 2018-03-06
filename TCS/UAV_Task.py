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
import time
from datetime import datetime

# import mraa
import sys
sys.path.append('/usr/local/lib/i386-linux-gnu/python2.7/site-packages/')


class vector3(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class Task_GOTO_Local(object):
    def __init__(self, setpoint_publish):# pointer "self" equal to pointer "this"
        self.setpoint_pub = setpoint_publish
        self.current_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
            								SP.PoseStamped, self._local_position_callback)
        self.current = vector3()
        self.msg = mavros.setpoint.PoseStamped(
            									header=mavros.setpoint.Header(
                								frame_id="att_pose",
                								stamp=rospy.Time.now()),
                							  )     
        self.x = 0
        self.y = 0
        self.z = 0
        # default precision is 0.5
        self.precision = 0.5    

    def goto(self, x=0, y=0, z=0, xyz=None):
        if (xyz==None):
            self.x = x
            self.y = y
            self.z = z
        else:
            self.x = xyz.x
            self.y = xyz.y
            self.z = xyz.z
        self._set_pose(self.msg, x, y, z)
        self.setpoint_pub.publish(self.msg)
        
	#uncalled function
    def check_task(self):
        if self.is_reached(self.current, self.msg):
            return True
        else:
            return False

    def is_reached(self, current, setpoint):
        if (abs(current.x-setpoint.pose.position.x) < self.precision and
            abs(current.y-setpoint.pose.position.y) < self.precision and
            abs(current.z-setpoint.pose.position.z) < self.precision):
            print "Point reached!"
            return True
        else:
            return False

    def _set_pose(self, pose, x, y, z):
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.header=mavros.setpoint.Header(frame_id="att_pose",stamp=rospy.Time.now())
                    
	#callback function for distance current_sub(topic:mavros/local_position/pose)
    def _local_position_callback(self,topic):
        self.current.x = topic.pose.position.x
        self.current.y = topic.pose.position.y
        self.current.z = topic.pose.position.z
	#uncalled function
    def set_precision(self, value):
        self.precision = value
	#uncalled function
    def get_setpoint_pub(self):
        return self.setpoint_pub
	#uncalled function
    def get_current(self):
        return self.current


class Task_Stay(Task_GOTO_Local):
    def __init__(self, setpoint_publish):
        super(Task_Stay, self).__init__(setpoint_publish)#???
        self.timer = rospy.Time.now()
        self.stay_pose = vector3();
        self.timer_ready = False
        self.duration = 0

    def stay_at_time(self, duration):
        # if timer is not being reset, reset the timer
        if (self.init_ready==False):
            print "First enter stay stay_at_time"
            self.timer = rospy.Time.now()
            self.init_ready = True
            self.stay_pose.x = self.current.x
            self.stay_pose.y = self.current.y
            self.stay_pose.z = self.current.z         

        self.goto(self.stay_pose.x,self.stay_pose.y,self.stay_pose.z)
        self.duration = duration

    def stay(self):
        if(self.init_ready==False):
            self.init_ready = True
            # need hard copy here because simple self.stay_pose = self.current will give reference to 
            # self.stay_pose and then self.stay_pose will change following the self.current
            self.stay_pose.x = self.current.x
            self.stay_pose.y = self.current.y
            self.stay_pose.z = self.current.z

        self.goto(self.stay_pose.x, self.stay_pose.y,self.stay_pose.z)

    def reset_stay(self):
        # need to reset say flag after use
        self.init_ready = False
        
	#uncalled function
    def check_task(self):
        if (rospy.Time.now() - self.timer > rospy.Duration(self.duration)):
            return True
        else:
            return False

    def print_current(self):
        print "x = {} y = {} z = {}".format(self.current.x, self.current.y, self.current.z)


class Task_GOTO_GPS(object):
    '''
    This is task that guide the UAV to GPS position
    '''
    pass


class API_XBee(object):
    '''
    This is the interface to the XBee via simple transparency grammer
    '''
    def __init__(self, PAN, ID):
        self.pan = PAN
        self.ID = ID
    def read(self):
        pass
    def _setID(self):
        pass
    def _setTargetID(self):
        pass
    def _serialize(self):
        pass
    def _deserialize(self):
        pass
    pass

class API_AirTemp(API_XBee):
    '''
    This is the interface to air temperature in ground data logger
    '''
    def read(self):
        pass
    def _serialize(self):
        pass
    def _deserialize(self):
        pass
    pass

class API_SoilTemp(API_XBee):
    '''
    This is the interface to Soil moisture sensor from ground data logger
    '''
    def read(self):
        pass
    def _serialize(self):
        pass
    def _deserialize(self):
        pass
    pass

class API_Moisture(API_XBee):
    '''
    This is interface to soil moisture sensor from  ground data logger
    '''
    def read(self):
        pass
    def _serialize(self):
        pass
    def _deserialize(self):
        pass
    pass

