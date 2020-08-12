#!/bin/bash

import os
import roslaunch
import rospy
import rosnode
import numpy as np
from std_msgs.msg import String, Float32

RADIUS = 0.075
yaml_text = "image: map_pgm_%d.pgm\nresolution: 0.15\norigin: [-3.75, 0.0, 0]\noccupied_thresh: 0.50\nfree_thresh: 0.50\nnegate: 0"

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# whenever listener node receives a message, update duration
curr_duration = 0.0
trial_running = True
def callback(data):
    global curr_duration
    global trial_running

    if data.data == -1.0:
        trial_running = False
    else:
        curr_duration = float(data.data)
        #print("%f" % curr_duration)

# create node to listen to duration topic
rospy.init_node('duration_listener', anonymous=True)
rospy.Subscriber('duration', Float32, callback)

# input: row and column from the occupancy grid
# output: their corresponding position in the gazebo world
def path_coord_to_gazebo_coord(x, y):
    r_shift = -RADIUS - (25 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)


for num in range(0, len(os.listdir('../worlds'))):
    # write the map number to the .yaml file
    yaml_file = open('../maps/trial_map.yaml', 'w')
    yaml_file.write(yaml_text % (num))
    yaml_file.close()
    
    curr_duration = 0.0
    trial_running = True

    # start and end points are currently sent as coordinates in jackal's c-space
    # with a cylinder radius of 0.075
    path = np.load('../Generated Paths/path_%d.npy' % num)
    path_start = path[0]
    path_end = path[len(path)-1]

    start_x, start_y = path_coord_to_gazebo_coord(path_start[0], path_start[1])
    goal_x, goal_y = path_coord_to_gazebo_coord(path_end[0], path_end[1])

    # end point is currently provided in c-space, so we need to add in more distance
    # for it to be in the obstacle space
    # TODO - remove once start & end points are in obstacle space
    goal_y += 3 * RADIUS * 2
    start_y -= 1

    world_name = 'world_%d.world' % num

    args_list = ['../launch/time_trial.launch', 'world_name:=$(find jackal_timer)/worlds/' + world_name, 'gui:=false', 'start_x:=' + str(start_x), 'start_y:=' + str(start_y), 
        'goal_x:=' + str(goal_x), 'goal_y:=' + str(goal_y), 'config:=front_laser']
    lifelong_args = args_list[1:]
    launch_files = [(roslaunch.rlutil.resolve_launch_arguments(args_list)[0], lifelong_args)]

    # launch the launch file
    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    parent.start()

    while trial_running:
        pass

    parent.shutdown()
