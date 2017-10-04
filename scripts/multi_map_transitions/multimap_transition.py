#!/usr/bin/env python

# Copyright (c) 2014 Mohit Shridhar, David Lee

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.i

#Modified by: Jose Mayoral Banos

import roslib
import rospy
import actionlib
from multi_map_navigation.msg import *
from std_msgs.msg import String

class MultiMapControl:
    def __init__(self):
        self.ac_server = actionlib.SimpleActionServer("custom", MultiMapServerAction, execute_cb=self.call_multiMap, auto_start=False)
        rospy.loginfo("Initializing Multimap Transition")
        # This relies on the 'dynamic_gazebo_models' package
        self.pub_map = rospy.Publisher('multi_map_server/map_name', String, queue_size=1)
        self.ac_server.start()
        rospy.loginfo("Started actionlib server")

    def call_multiMap(self, msg):
        self.pub_map.publish(msg.map_name)
        rospy.loginfo("Waiting transition to finish...")
        self.ac_server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node("custom")
    blast = MultiMapControl()
    rospy.spin()
