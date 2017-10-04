#!/usr/bin/env python

import roslib
roslib.load_manifest('multi_map_navigation')
import rospy
import subprocess
import sys
from nav_msgs.msg import *
from geometry_msgs.msg import *
import tf

rospy.init_node("amcl_restart")

proc = None
shutdown = False
cached_pose = None
pose_out = None

def poseCb(msg):
    global pose_out
    global cached_pose
    cached_pose = msg
    pose_out.publish(msg)
    

def mapCb(msg):
    msg = False #Not used, clear to reduce possible bugs
    global proc
    global shutdown
    global cached_pose
    rospy.loginfo("Restarting AMCL")
    if (proc):
        shutdown = True
        try:
            proc.terminate()
            proc.wait()
        except:
            rospy.logerr("Exception when trying to shut down")
    #write parameters
    pose = cached_pose
    if (pose):
        pose = pose.pose.pose
        rospy.set_param('amcl/initial_pose_x', pose.position.x)
        rospy.set_param('amcl/initial_pose_y', pose.position.y)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion( \
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        rospy.set_param('amcl/initial_pose_a', yaw)
    proc = subprocess.Popen(["roslaunch", sys.argv[1]])
    shutdown = False

if (len(sys.argv) < 2):
    rospy.logerr("You must specify a roslaunch file as an argument.")
else:
    proc = subprocess.Popen(["roslaunch", sys.argv[1]])
    secondary = rospy.Subscriber("/map", OccupancyGrid, mapCb)
    pose_out = rospy.Publisher("/amclinitialpose", PoseWithCovarianceStamped)
    sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, poseCb)
    listener = tf.TransformListener(True, rospy.Duration(100))
    bad_counter = 0
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time())
            bad_counter = 0
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            bad_counter = bad_counter + 1
            rospy.logwarn("Failed to get robot transform. Resetting pose")
            pose = cached_pose
            if (pose):
                rospy.logwarn("Had to re-publish pose")
                pose_out.publish(pose)
            if (bad_counter > 4):
                rospy.logwarn("AMCL seems to have died - restarting")
                mapCb(False)
                bad_counter = 0
            
            rospy.sleep(3.0)
    if (not shutdown):
        shutdown = True
        proc.terminate()


