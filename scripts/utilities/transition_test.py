#!/usr/bin/python

import roslib
roslib.load_manifest('multi_map_navigation')
import rospy
import actionlib
import sys
import yaml
from multi_map_navigation.msg import *
from multi_map_navigation.srv import *


class TestClient():
    def __init__(self, name, start, end, wormhole):
        self.action_client = actionlib.SimpleActionClient(name, MultiMapNavigationTransitionAction)
        self.action_client.wait_for_server()
        goal = MultiMapNavigationTransitionGoal()
        goal.start = start
        goal.end = end
        goal.wormhole = wormhole
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()


if (__name__ == "__main__"):
   rospy.init_node("multi_map_navigation")
   name = sys.argv[1]
   start = 0
   end = 0
   wormhole = ""
   bad = False
   if (len(sys.argv) > 3):
       start = int(sys.argv[2].strip())
       end = int(sys.argv[3].strip())
   if (len(sys.argv) > 5):
       try:
           f = open(sys.argv[4].strip(), 'r')
           text = f.read()
           data = yaml.load(text)
           if (not "wormholes" in data):
               print "No wormholes in the map"
               bad = True
           for i in data["wormholes"]:
               if ("name" in i):
                   if (i["name"].strip() == sys.argv[5].strip()):
                       wormhole = yaml.dump(i)
                       print "Sending wormhole: ", wormhole
                       if ("locations" in i):
                           print "Starting location: ", i["locations"][start]
                           print "Ending location: ", i["locations"][end]
       except:
           print "Error attempting to load YAML: " + sys.argv[4].strip()
           bad = True
       
   if (not bad):
       test = TestClient(name, start, end, wormhole)

