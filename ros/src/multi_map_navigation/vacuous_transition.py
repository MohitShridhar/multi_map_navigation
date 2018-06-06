#!/usr/bin/python

import roslib
roslib.load_manifest('multi_map_navigation')
import rospy
import actionlib
from multi_map_navigation.msg import *

server = None

def vacuous(data):
    global server
    ROS_INFO("Transition Server :)")
    server.set_succeeded(MultiMapNavigationTransitionResult())
