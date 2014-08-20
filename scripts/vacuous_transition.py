#!/usr/bin/python

import roslib
roslib.load_manifest('multi_map_navigation')
import rospy
import actionlib
from multi_map_navigation.msg import *

server = None

def vacuous(data):
    global server
    server.set_succeeded(MultiMapNavigationTransitionResult())

if (__name__ == '__main__'):
   client = sys.argv[1]
   rospy.init_node(client)
   server = actionlib.SimpleActionServer(client, MultiMapNavigationTransitionAction, execute_cb = vacuous, auto_start = False)
   server.start()
   rospy.spin()
