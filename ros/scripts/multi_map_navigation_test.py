#!/usr/bin/python

import roslib
roslib.load_manifest('multi_map_navigation')
import rospy
import actionlib
import sys
from multi_map_navigation.msg import *
from multi_map_navigation.srv import *


class TestClient():
    def __init__(self, robot_namespace, xp, yp, map):


        self.action_client = actionlib.SimpleActionClient("/" + robot_namespace + "/multi_map_navigation/move", MultiMapNavigationAction)
        rospy.loginfo("Waiting for server...")
        
        if not self.action_client.wait_for_server():
          rospy.logerr("Server not available. Check that " + robot_namespace + " actually exists")
          return None
        
        goal = MultiMapNavigationGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = robot_namespace + "/map"
        goal.target_pose.pose.position.x = xp
        goal.target_pose.pose.position.y = yp
        goal.target_pose.pose.orientation.w = 1.0
        goal.goal_map = map
        self.action_client.send_goal(goal)
        rospy.loginfo("Sent goal")

        self.action_client.wait_for_result()


if (__name__ == "__main__"):
   rospy.init_node("multi_map_navigation_test")
   robot_namespace = sys.argv[1]
   xp = float(sys.argv[2])
   yp = float(sys.argv[3])
   map = sys.argv[4] 
   test = TestClient(robot_namespace, xp, yp, map)

