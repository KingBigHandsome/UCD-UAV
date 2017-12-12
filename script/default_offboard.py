#!/usr/bin/env python

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
from UAV_Task import *
# import utilities
import thread
import math
import sys
import signal

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


current_pose = vector3()
UAV_state = mavros_msgs.msg.State()



def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided



def _setpoint_position_callback(topic):
    pass

def _set_pose(pose, x, y, z):
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now())



def is_reached(current, setpoint):
    if (abs(current.x-setpoint.pose.position.x) < 0.5 and
        abs(current.y-setpoint.pose.position.y) < 0.5 and
        abs(current.z-setpoint.pose.position.z) < 0.5):
            print "Point reached!"
        return True
    else:
        return False



def main():
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(10)
    mavros.set_namespace('/mavros')


    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),
        mavros_msgs.msg.State, _state_callback)
    # /mavros/local_position/pose
    # local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
    #     SP.PoseStamped, _local_position_callback)
    # /mavros/setpoint_raw/target_local
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
        mavros_msgs.msg.PositionTarget, _setpoint_position_callback)

    # setup publisher
    # /mavros/setpoint/position/local
    setpoint_local_pub =  mavros.setpoint.get_pub_position_local(queue_size=10)

    # setup service
    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  

    setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )


    # wait for FCU connection
    while(not UAV_state.connected):
        rate.sleep()

    # create task instance
    task_goto = Task_GOTO_Local(setpoint_local_pub)
    task_stay = Task_Stay(setpoint_local_pub)
    # initialize the setpoint
    setpoint_msg.pose.position.x = 0
    setpoint_msg.pose.position.y = 0
    setpoint_msg.pose.position.z = 3
    	
    mavros.command.arming(True)

    # send 100 setpoints before starting
    for i in range(0,50):
        setpoint_local_pub.publish(setpoint_msg)
        rate.sleep()

    set_mode(0,'OFFBOARD')

    last_request = rospy.Time.now()

    step = 1
    task_done = False

    while(True):
        # print "Entered whiled loop"
        if( UAV_state.mode != "OFFBOARD" and
            (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            if( set_mode(0,'OFFBOARD').success):
                print "Offboard enabled"
            last_request = rospy.Time.now()
        else:
            if(not UAV_state.armed and
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if( mavros.command.arming(True)):
                    print "Vehicle armed"
                last_request = rospy.Time.now()
        # print "Before Task loop"
        if (UAV_state.armed):
            # print "Now we are in step {0}".format(step)
            if (step == 1):
                task_goto.goto(5,5,8.1)
                task_done = task_goto.check_task()

            elif (step == 2):
                # stay at current position for 10 seconds
                task_stay.stay_at_time(10.0)
                task_done = task_stay.check_task()
            elif (step == 3):
                task_goto.goto(-10,0,5.1)
                task_done = task_goto.check_task()
            elif (step == 4):
                task_goto.goto(0,0,5.1)
                task_done = task_goto.check_task()
            else:
                while (UAV_state.mode != "AUTO.LAND"):
                    set_mode(0,'AUTO.LAND')
                    rate.sleep()
                return 0
                     # Exit program
            if (task_done):
                step+=1
                task_stay.reset_stay()
            
        #setpoint_local_pub.publish(setpoint_msg)

        rate.sleep()
    return 0




if __name__ == '__main__':
    main()
