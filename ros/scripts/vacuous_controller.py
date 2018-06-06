#!/usr/bin/env python

from multi_map_navigation.vacuous_transition import vacuous
import rospy

if (__name__ == '__main__'):
   client = sys.argv[1]
   rospy.init_node(client)
   server = actionlib.SimpleActionServer(client, MultiMapNavigationTransitionAction, execute_cb = vacuous, auto_start = False)
   server.start()
   rospy.spin()
