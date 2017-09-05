#!/usr/bin/env python

import roslib
import rospkg
import rospy
import actionlib
import sys, os
from tf2_msgs.msg import *

from nav_msgs.msg import *
from nav_msgs.srv import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
import yaml
from multi_map_navigation.msg import *
from multi_map_navigation.srv import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import math
import tf
import random

def calc_distance(l1, l2):
    dx = l1[0] - l2[0]
    dy = l1[1] - l2[1]
    return math.sqrt(dx * dx + dy * dy)

class MultiMapManager(object):
    def __init__(self):
        self.ready = False
        self.setup_namespaces()
        self.listener = tf.TransformListener()
        self.list_maps = ["kitchen", "living-room",'dining-room','shelf']
        self.current_map_name_pub = rospy.Publisher("map_name", String, queue_size=1)
        self.wormhole_marker_pub = rospy.Publisher('wormhole_marker', MarkerArray, queue_size=1)
        self.n_markers = 0

        rospy.loginfo("loading map")

        self.definition_file = None
        if rospy.has_param('~definition_file'):
            self.definition_file = rospy.get_param("~definition_file")
        else:
            rospy.logerr("You must specify a definition_file")
            return

        if (not self.loadyaml(self.definition_file)):
            return
        #rospy.loginfo("Waiting for position")

        self.get_robot_position()
        print "after position"

        self.ready = True

        rospy.loginfo("Starting")

    def setup_namespaces(self):

        self.robot_namespace = "" # default namespace (not recommended)
        self.base_frame = "base_link"
        if rospy.has_param('~base_frame'):
            self.base_frame = rospy.get_param("~base_frame")

        self.base_frame = "/" + self.base_frame
        rospy.loginfo("base_frame of " + self.robot_namespace + " set as " + self.base_frame)

        return None

    def get_robot_position(self):
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform("/map", self.base_frame, rospy.Time(), rospy.Duration(100))
                (trans,rot) = self.listener.lookupTransform('/map', self.base_frame, rospy.Time())
                return [trans[0], trans[1]]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logwarn("Failed to get robot transform")
                rospy.sleep(0.1)
        print "returning None"
        return None

    def get_robot_rotation(self):
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform("/map", self.base_frame, rospy.Time(), rospy.Duration(100))
                (trans,rot) = self.listener.lookupTransform('/map',self.base_frame, rospy.Time())
                return rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logwarn("Failed to get robot transform")
                rospy.sleep(0.1)
        return None

    def publish_current_map_name(self):
        self.current_map_name_pub.publish(self.current_map)

    def publish_markers(self):
        n_markers = 0
        marker_array = MarkerArray()
        for i in self.wormholes:
            loc = False
            #print "Current wormhole", i["name"]
            if i["name"] == self.current_map:
                for j in i["locations"]:
                    wormhole_marker = Marker()
                    wormhole_marker.header.frame_id = "/map"
                    wormhole_marker.header.stamp = rospy.get_rostime()
                    wormhole_marker.ns = "multimna"
                    wormhole_marker.type = Marker.CYLINDER
                    wormhole_marker.action = Marker.MODIFY
                    wormhole_marker.id = n_markers
                    wormhole_marker.pose.position.x = j["position"][0]
                    wormhole_marker.pose.position.y = j["position"][0]
                    wormhole_marker.pose.position.z = 0.25
                    wormhole_marker.pose.orientation.x = 0.0
                    wormhole_marker.pose.orientation.y = 0.0
                    wormhole_marker.pose.orientation.z = 0.0
                    wormhole_marker.pose.orientation.w = 1.0
                    if "radius" in i:
                        wormhole_marker.scale.x = i["radius"] * 2.0
                        wormhole_marker.scale.y = i["radius"] * 2.0
                    else:
                        wormhole_marker.scale.x = 0.5
                        wormhole_marker.scale.y = 0.5
                    wormhole_marker.scale.z = 1
                    wormhole_marker.color.a = 0.6
                    wormhole_marker.color.r = 0.0
                    wormhole_marker.color.g = 0.0
                    wormhole_marker.color.b = 1.0

                    marker_array.markers.append(wormhole_marker)
                    n_markers = n_markers + 1

            self.wormhole_marker_pub.publish(marker_array)


        if (self.n_markers > n_markers):
            for i in range(n_markers, self.n_markers):
                wormhole_marker = Marker()
                wormhole_marker.action = Marker.DELETE
                wormhole_marker.id = self.n_markers

                waiting_area_marker = Marker()
                waiting_area_marker = Marker.DELETE
                waiting_area_marker.id = self.n_markers

                self.wormhole_marker_pub.publish(wormhole_marker)
        self.n_markers = n_markers

    def loadyaml(self, filename):
        print "LOADING YAML"
        try:
            print "file:", filename
            f = open(filename, 'r')
            text = f.read()
            data = yaml.load(text)
        except:
            rospy.logerr("Error attempting to load YAML: " + filename)
            return False
        if (not "maps" in data):
            rospy.logerr("YAML file: " + filename + " contains no maps")
            return False
        if (not "wormholes" in data):
            rospy.logerr("YAML file: " + filename + " contains no wormholes")
            return False
        if (not "start_map" in data):
            rospy.logerr("YAML file: " + filename + " contains no start_map")
            return False

        self.current_map = data["start_map"]
        self.publish_current_map_name()

        self.maps = {}
        self.map_north = 1.5707963267948966
        for i in data["maps"]:
            print "maps available", i
            if (not "name" in i):
                rospy.logerr("YAML file: " + filename + " contains an invalid map with no name")
                return False
            self.maps[i["name"]] =  [i["name"]]

        self.wormholes = data["wormholes"]
        n = 0
        wh_names = []
        for i in self.wormholes:
            print "wormholes available"
            if (not "name" in i):
                rospy.logerr("YAML file: " + filename + " contains an invalid wormhole which is missing a name")
                return False
            if (i["name"] in wh_names):
                rospy.logerr("Duplicate wormholes: " + i["name"])
                return False
            wh_names.append(i["name"])
            if (not "locations" in i):
                rospy.logerr("YAML file: " + filename + " contains an invalid wormhole which is missing locations")
                return False
            for loc in i["locations"]:
                if (not "map" in loc):
                    rospy.logerr("YAML file: " + filename + " contains an invalid location which is missing a map")
                    return False
                if (not "position" in loc):
                    rospy.logerr("YAML file: " + filename + " contains an invalid location which is missing a position")
                    return False
        self.start_map = data["start_map"]

        return True


class MultiMapNavigationNavigator():
    def __init__(self, manager):
        self.manager = manager
        self.robot_namespace = manager.robot_namespace

        move_base_name = "move_base"
        if rospy.has_param('~move_base_action'):
            move_base_name = rospy.get_param("~move_base_action")
        self.move_base = actionlib.SimpleActionClient(move_base_name, MoveBaseAction)
        self.move_base.wait_for_server()
        self.action_server = actionlib.SimpleActionServer("multi_map_navigation/move", MultiMapNavigationAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        print "sevice initialize"
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        while not self.manager.ready:
            rospy.sleep(1.0)

        self.action_server.start()

        self.preempt_goal = False
        self.cancel_goals_sub = rospy.Subscriber("/cancel_all_goals", Bool, self.cancel_cb)

    def cancel_cb(self, msg):
        self.preempt_goal = msg.data

    def execute_cb(self, goal):
        #print self.manager.current_map
        #print goal.goal_map

        #Create a graph of all the wormholes. The nodes are the locations.
        print "execute_cb"
        graph = {'start': {}, 'end': {}}


        for w in self.manager.wormholes:
            #The cost of moving through a wormhole is nothing. In the future, this
            #could be non-zero if there is an object such as a door or elevator that
            #could slow the robot's motion.
            for l in range(0, len(w["locations"])):
                direction = "double"
                if ("direction" in w["locations"][l]):
                    direction = w["locations"][l]["direction"]
                traverse = {}
                if (direction == "double" or direction == "entrance"):
                    for j in range(0, len(w["locations"])):
                        direction = "double"
                        if ("direction" in w["locations"][j]):
                            direction = w["locations"][j]["direction"]
                        if (direction == "exit" or direction == "double"):
                            traverse[str(j) + "_" + w["name"]] = 100.0
                graph[str(l) + "_" + w["name"]] = traverse

        robot_pos = self.manager.get_robot_position()

        print graph

        #Create the graph for each of the wormholes
        for w in self.manager.wormholes:
            for l in range(0, len(w["locations"])):
                #loop through the locations of the wormhole and add links
                #to the start and end as well as other locations
                ll = w["locations"][l]
                for m in self.manager.wormholes:
                    for j in range(0, len(m["locations"])):
                        jl = m["locations"][j]
                        if (ll["map"] == jl["map"]):
                            #We only need to do this one-way because the other side
                            #is done automatically in other iterations of the loop.
                            #For now we use eucleadian distance. This works but it
                            #would be better if we used actual planner results.
                            graph[str(l) + "_" + w["name"]][str(j) + "_" + m["name"]] \
                                = calc_distance(ll["position"], jl["position"])

                if (ll["map"] == goal.goal_map):
                    dist = calc_distance([goal.target_pose.pose.position.x,
                                          goal.target_pose.pose.position.y], ll["position"])
                    graph[str(l) + "_" + w["name"]]["end"] = dist
                    graph["end"][str(l) + "_" + w["name"]] = dist
                if (ll["map"] == self.manager.current_map):
                    dist = calc_distance(robot_pos, ll["position"])
                    graph[str(l) + "_" + w["name"]]["start"] = dist
                    graph["start"][str(l) + "_" + w["name"]] = dist

        if (goal.goal_map == self.manager.current_map):
            dist = calc_distance([goal.target_pose.pose.position.x,
                                  goal.target_pose.pose.position.y], robot_pos)
            graph["start"]["end"] = dist
            graph["end"]["start"] = dist

        print "GRAPH ", graph
        path = self.shortest_path(graph, "start", "end")[1:] #skip "start"
        print "PATH ", path

        offset = []
        old_north = 0.0

        old_pos = [None, None]
        old_angle = None

        while (path[0] != "end"):
            print path[0]

            #wormhole
            name = path[0][path[0].find("_") + 1:]
            #print name
            wormhole = None
            for i in self.manager.wormholes:
                if (i["name"] == name):
                    wormhole = i
            #print wormhole
            location = wormhole["locations"][int(path[0].split("_")[0])]
            pos = location["position"]
            mapname = location["map"]
            north = self.manager.map_north
            wormhole_type = "normal"
            wormhole_goal = None
            if (len(path) > 1):
                if (path[0][path[0].find("_") + 1:] == path[1][path[0].find("_") + 1:]):
                    #wormhole_type = wormhole["type"]
                    wormhole_goal = MultiMapNavigationTransitionGoal()
                    wormhole_goal.wormhole = yaml.dump(wormhole)
                    wormhole_goal.start = int(path[0].split("_")[0])
                    wormhole_goal.end = int(path[1].split("_")[0])

            angle = 0
            radius = None
            if "radius" in wormhole:
                #This is in fact not required. Some wormholes have zero radius
                #in which case the robot must drive as close as possible to the
                #wormhole. Examples include doors and elevators.
                radius = wormhole["radius"]
            if "angle" in location:
                #This requires the robot to take a fixed angle position. This is
                #used for things like doors and elevators. It won't work if a
                #radius is specified
                angle = location["angle"]
                if (radius != None):
                    rospy.logwarn("Angle ignored because radius specified")

            #We have switched maps in the wormhole
            if (mapname != self.manager.current_map):
                #Create and publish the new pose for AMCL
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "/map"
                msg.header.stamp = rospy.get_rostime()

                offset_angle = 0.0
                offset_radius = 0.0
                if (radius):
                    offset_angle = math.atan2(offset[1], offset[0])
                    offset_radius = math.sqrt(offset[0] * offset[0] + offset[1] * offset[1])

                    offset_angle += north - old_north

                msg.pose.pose.position.x = pos[0] + math.cos(offset_angle) * offset_radius
                msg.pose.pose.position.y = pos[1] + math.sin(offset_angle) * offset_radius
                msg.pose.pose.position.z = 0.0

                roll, pitch, yaw = tf.transformations.euler_from_quaternion( \
                    self.manager.get_robot_rotation())
                #print yaw
                yaw += north - old_north
                if (angle):
                    yaw = angle
                quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
                msg.pose.pose.orientation.x = quat[0]
                msg.pose.pose.orientation.y = quat[1]
                msg.pose.pose.orientation.z = quat[2]
                msg.pose.pose.orientation.w = quat[3]

                msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.06853891945200942]

                #print msg
                # self.pose_pub.publish(msg)
                #Select the new map
                #self.manager.select_map(mapname)

                # emptySrv = std_srvs.srv.Empty()
                # rospy.wait_for_service(self.robot_namespace + "/global_localization")

                # try:
                #     init_amcl = rospy.ServiceProxy(self.robot_namespace + "/global_localization", Empty)
                #     init_amcl()
                # except:
                #     rospy.logerr("Could not re-initialize AMCL")

                # self.pose_pub.publish(msg)

                rospy.sleep(2) #FIXME: come up with an alternative approach to know when AMCL is done

                offset = self.manager.get_robot_position()
                offset[0] = offset[0] - pos[0]
                offset[1] = offset[1] - pos[1]

                rospy.loginfo("Wait for movebase")
                self.move_base.wait_for_server()
                rospy.loginfo("Done")

            elif (angle != old_angle or old_pos[0] != pos[0] or old_pos[1] != pos[1]):
                #We need to do driving to get to the next wormhole
                #print pos
                #Create the goal for the next waypoint (at the target)
                rospy.loginfo("Heading towards a wormhole")
		"""
                if (wormhole["type"] == "elevator_blast"):
                    wasGoalSuccessful = self.go_to_waiting_point(location["waiting_point"])
                    if not wasGoalSuccessful:
                        rospy.loginfo("Goal aborted!")
                        return None;

                    self.target_elevator(location["floor"], wormhole["name"]) # call elevator to current floor
		"""
                msg = MoveBaseGoal()
                msg.target_pose.header.stamp = rospy.get_rostime()
                msg.target_pose.header.frame_id = self.robot_namespace + "/map"
                msg.target_pose.pose.position.x = pos[0]
                msg.target_pose.pose.position.y = pos[1]
                msg.target_pose.pose.position.z = 0
                quat = tf.transformations.quaternion_from_euler(0, 0, angle)
                msg.target_pose.pose.orientation.x = quat[0]
                msg.target_pose.pose.orientation.y = quat[1]
                msg.target_pose.pose.orientation.z = quat[2]
                msg.target_pose.pose.orientation.w = quat[3]

                wasGoalSuccessful = self.go_to_goal(msg, radius)
                if not wasGoalSuccessful:
                    rospy.loginfo("Goal aborted!")
                    return None;

                rospy.loginfo("Done move_base")
                offset = self.manager.get_robot_position()
                offset[0] = offset[0] - pos[0]
                offset[1] = offset[1] - pos[1]
            else:
                rospy.loginfo("Skipped move base because the goal location is the current location")

	    """
            if (wormhole_type == "elevator_blast" and wormhole_goal != None):
                rospy.loginfo("Transition: Elevator Blast")
                next_floor = self.find_target_floor(wormhole, goal.goal_map)
                self.target_elevator(next_floor, wormhole["name"])
	
            elif (wormhole_type != "normal" and wormhole_goal != None):
                rospy.loginfo("Transition: " + str(wormhole_type))
                cli = self.manager.transition_action_clients[wormhole_type]

                #print wormhole_goal
                cli.send_goal(wormhole_goal)
                cli.wait_for_result()
	    """
            rospy.loginfo("Transition: " + str(wormhole_type))
            #cli = self.manager.transition_action_clients[wormhole_type]

            #print wormhole_goal
            #cli.send_goal(wormhole_goal)
            #cli.wait_for_result()
            
            print "DONE AFTER TRANSACTION"
            old_pos = pos
            old_angle = angle
            old_north = north

            path = path[1:]
            print path


        #Get to the end point
        msg = MoveBaseGoal()
        msg.target_pose = goal.target_pose

        wasGoalSuccessful = self.go_to_goal(msg, None)
        if not wasGoalSuccessful:
            rospy.loginfo("Goal aborted!")
            return None

        self.action_server.set_succeeded(MultiMapNavigationResult())

    def find_target_floor(self, wormhole, goal_map):
        target_loc = None

        for i in wormhole["locations"]:
            if (i["map"] == goal_map):
                target_loc = i

        return target_loc["floor"]

    def target_elevator(self, target_floor, elevator_id):
        elevator_target = multi_map_navigation.msg.MultiMapNavigationTargetElevatorGoal()
        elevator_target.elevatorTargetFloor = target_floor
        elevator_target.elevatorId = elevator_id

        client = self.manager.transition_action_clients["elevator_blast"]
        client.send_goal_and_wait(elevator_target, execute_timeout=rospy.Duration.from_sec(600.0),  preempt_timeout=rospy.Duration.from_sec(700.0)) # maximum wait time for elevator: ~10 minutes

    def go_to_waiting_point(self, waiting_point):

        rospy.loginfo("But heading to waiting area first")

        pos_str = waiting_point[0]
        waiting_pose = [float(x) for x in pos_str.split()]
        print waiting_pose

        msg = MoveBaseGoal()
        msg.target_pose.header.stamp = rospy.get_rostime()
        msg.target_pose.header.frame_id = "/map"
        msg.target_pose.pose.position.x = waiting_pose[0]
        msg.target_pose.pose.position.y = waiting_pose[1]
        msg.target_pose.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, waiting_pose[2])
        msg.target_pose.pose.orientation.x = quat[0]
        msg.target_pose.pose.orientation.y = quat[1]
        msg.target_pose.pose.orientation.z = quat[2]
        msg.target_pose.pose.orientation.w = quat[3]

        wasWaitingGoalSuccessful = self.go_to_goal(msg, 0.15) # waiting_point radius is set as 0.15 m by default
        if not wasWaitingGoalSuccessful:
            return False # goal was aborted before reaching the waiting area

        rospy.loginfo("Reached waiting area. Waiting for controller...")
        return True;

    def go_to_goal(self, msg, radius=None):
        print msg
        #print "Wait for move base"
        bad = True
        bad_count = 0
        while not rospy.is_shutdown() and bad:
            rospy.loginfo("Send move base goal radius: " + str(radius))
            self.move_base.send_goal(msg)

            while (self.move_base.get_state() == GoalStatus.PENDING or \
                        self.move_base.get_state() == GoalStatus.ACTIVE) and \
                        not rospy.is_shutdown():

                rospy.sleep(0.1)

                if (self.preempt_goal == True):
                    self.action_server.set_aborted(True)
                    self.move_base.cancel_all_goals()
                    return False

                if (radius != None):
                    if calc_distance((msg.target_pose.pose.position.x,
                                      msg.target_pose.pose.position.y),
                                     self.manager.get_robot_position()) < radius:
                        bad = False
                        self.move_base.cancel_goal()
                        break

            self.move_base.wait_for_result()
            if (self.move_base.get_state() == GoalStatus.SUCCEEDED or bad == False):
                bad = False
            else:
                rospy.sleep(1.0)
            if bad:
                bad_count = bad_count + 1
                if (bad_count > 1):
                    bad_count = 0
                    rospy.logerr("Movebase is not responding. Try rotating to random positions")
                    bad = True
                    while not rospy.is_shutdown() and bad:
                        pos = self.manager.get_robot_position()
                        angle = random.uniform(-math.pi, math.pi)

                        spin = MoveBaseGoal()
                        spin.target_pose.header.stamp = rospy.get_rostime()
                        spin.target_pose.header.frame_id = "/map"
                        spin.target_pose.pose.position.x = pos[0]
                        spin.target_pose.pose.position.y = pos[1]
                        spin.target_pose.pose.position.z = 0
                        quat = tf.transformations.quaternion_from_euler(0, 0, angle)
                        spin.target_pose.pose.orientation.x = quat[0]
                        spin.target_pose.pose.orientation.y = quat[1]
                        spin.target_pose.pose.orientation.z = quat[2]
                        spin.target_pose.pose.orientation.w = quat[3]
                        self.move_base.send_goal(spin)

                        while (self.move_base.get_state() == GoalStatus.PENDING or self.move_base.get_state() == GoalStatus.ACTIVE) and not rospy.is_shutdown():
                            rospy.sleep(0.1)
                            if (self.preempt_goal == True):
                                self.action_server.set_aborted(True)
                                self.move_base.cancel_all_goals()
                                return False

                        self.move_base.wait_for_result()
                        if (self.move_base.get_state() == GoalStatus.SUCCEEDED):
                            bad = False
                    bad = True #Make sure we are still bad for actual motion
        return True

    def shortest_path(self, graph, start, end):
        distances = {}
        predecessors = {}
        queue = {}
        queue[start] = 0

        while True:
            #find min
            vertex = None
            for v in queue:
                if (vertex == None):
                    vertex = v
                if (queue[v] < queue[vertex]):
                    vertex = v
            #Dijksta
            distances[vertex] = queue[vertex]
            if (vertex == end):
                break
            for option in graph[vertex]:
                length = distances[vertex] + graph[vertex][option]
                if option in distances:
                    if length < distances[vertex]:
                        print "Error"
                elif option not in queue or length < queue[option]:
                    queue[option] = length
                    predecessors[option] = vertex
            #Remove
            del queue[vertex]

        path = []
        while True:
            path.append(end)

            if (end == start): break
            end = predecessors[end]
        path.reverse()
        return path


def handle_reinit_call(req):
    # rospy.loginfo("Re-initializing Multi-Map navigator...")
    #manager.create_map_db()

    #if (not manager.loadyaml(manager.definition_file)):
        #rospy.logerr("Re-initialization FAILED. Could not load updated YAML file")
        #return Empty
    return Empty

if (__name__ == "__main__"):
    rospy.init_node('my_multi_map_navigation')
    manager = MultiMapManager()
    navigator = MultiMapNavigationNavigator(manager)

    reinit_service = rospy.Service('map_manager/reinit', ReinitManager, handle_reinit_call)
    print "after re_init"
    while not rospy.is_shutdown():
        manager.publish_markers()
        manager.publish_current_map_name()
        rospy.sleep(1.0)
m
