#!/usr/bin/env python

from multi_map_navigation.multimap_transition import MultiMapControl
import rospy

if __name__ == '__main__':
    rospy.init_node("elevator_blast")

    blast = MultiMapControl()
    rospy.spin()
