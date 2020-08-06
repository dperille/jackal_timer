#!/bin/bash

import roslaunch
import rospy
import rosnode
import numpy as np
from std_msgs.msg import String, Float32

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

args_list = ['../launch/time_trial.launch', 'gui:=true', 'start_x:=5', 'start_y:=0', 'goal_x:=10', 'goal_y:=0', 'config:=front_laser']
lifelong_args = args_list[1:]
launch_files = [(roslaunch.rlutil.resolve_launch_arguments(args_list)[0], lifelong_args)]

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
parent.start()

# whenever listener node receives a message, update duration
running_duration = 0.0
def callback(data):
    global running_duration
    running_duration = float(data.data)
    print("%f" % running_duration)

# create node to listen to duration topic
rospy.init_node('time_listener', anonymous=True)
rospy.Subscriber('duration', Float32, callback)
rospy.spin()

print("Final duration: %f" % running_duration)