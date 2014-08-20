#!/usr/bin/env python

import roslib
import rospy
import math
import tf
import yaml
from sensor_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from multi_map_navigation.msg import *
from trajectory_msgs.msg import *
import actionlib
import cv
from std_msgs.msg import Int32
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_msgs.msg import UInt32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib.packages
                
class ElevatorControl:
    def __init__(self):
        self.ac_server = actionlib.SimpleActionServer("elevator_blast", MultiMapNavigationTargetElevatorAction, execute_cb=self.do_elevator, auto_start=False)
        rospy.loginfo("Initializing Elevator Manager")

        self.elevatorId = "elevator1" # default elevator control
        self.sub_est_floor = rospy.Subscriber('/elevator_controller/' + self.elevatorId + '/estimated_current_floor', Int32, self.estimated_floor_cb)
        
        self.pub_active_elevators = rospy.Publisher('/elevator_controller/active', UInt32MultiArray, queue_size=10)
        self.pub_target_floor = rospy.Publisher('/elevator_controller/target_floor', Int32, queue_size=10)
        self.pub_door_opener = rospy.Publisher('/elevator_controller/door', UInt8, queue_size=2)

        self.curr_elevator_ref = std_msgs.msg.UInt32MultiArray()
        self.curr_elevator_ref.data = {1} 
        self.new_target = std_msgs.msg.Int32()
        self.open_doors_cmd = std_msgs.msg.UInt8()
        self.open_doors_cmd.data = 1

        self.ac_server.start()
        rospy.loginfo("Started actionlib server")

    def do_elevator(self, msg):
        self.new_target.data = msg.elevatorTargetFloor
        
        if (self.elevatorId != msg.elevatorId):
            self.elevatorId = msg.elevatorId
            self.sub_est_floor.shutdown()
            self.sub_est_floor = rospy.Subscriber('/elevator_controller/' + self.elevatorId + '/estimated_current_floor', Int32, self.estimated_floor_cb)

        self.curr_elevator_ref.data = {int(self.elevatorId.replace("elevator", ""))}

        self.pub_active_elevators.publish(self.curr_elevator_ref)

        rospy.loginfo("Waiting for elevator to finish...")
        while self.new_target.data != self.currElevatorPos:
            self.pub_target_floor.publish(self.new_target) # republish in case of data drop
            # pass

        self.pub_door_opener.publish(self.open_doors_cmd)

        rospy.loginfo("The elevator has arrived")
        self.ac_server.set_succeeded()

    def estimated_floor_cb(self, msg):
        self.currElevatorPos = msg.data

if __name__ == '__main__':
    rospy.init_node("elevator_blast")

    blast = ElevatorControl()
    rospy.spin()
    