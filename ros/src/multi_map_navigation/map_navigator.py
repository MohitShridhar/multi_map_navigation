import rospy
import math
import random
import tf
import yaml
import networkx as nx
import actionlib
import actionlib_msgs
import numpy
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty

import matplotlib.pyplot as plt

from nav_msgs.srv import *
from geometry_msgs.msg import *
from multi_map_navigation.msg import *
from multi_map_navigation.srv import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus

def calc_distance(l1, l2):
    dx = l1[0] - l2[0]
    dy = l1[1] - l2[1]
    return math.sqrt(dx * dx + dy * dy)

class MultiMapNavigationNavigator():
    def __init__(self, manager):
        self.manager = manager
        self.robot_namespace = manager.robot_namespace

        move_base_name = "move_base"
        self.robot_id = "Lucy"

        #Move Base Related
        if rospy.has_param('~move_base_action'):
            move_base_name = rospy.get_param("~move_base_action")

        if rospy.has_param('~robot_id'):
            self.robot_id = rospy.get_param("~robot_id")

        self.move_base = actionlib.SimpleActionClient(move_base_name, MoveBaseAction)
        self.move_base.wait_for_server()

        #This is the Server To Access MultiMapNavigation
        self.action_server = actionlib.SimpleActionServer("multi_map_navigation/move", MultiMapNavigationAction,
                                                          execute_cb=self.execute_cb, auto_start=False)

        #Relocalize in New Map
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        #Elevator Request
        self.elevator_request_pub = rospy.Publisher('elevator_request', elevatorRequest, queue_size=1)

        #Debug
        self.cuurent_floor = None

        self.current_goal_pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)

        while not self.manager.ready:
            pass


        #Elevator
        self.elevator_available = numpy.zeros(3, dtype=bool)

        for i in range(3):
            rospy.Subscriber('/elevator_controller/' + str(i) + '/available_for', String, self.availableElevatorFor,i)

        self.action_server.start()
        self.preempt_goal = False
        self.cancel_goals_sub = rospy.Subscriber("/cancel_all_goals", Bool, self.cancel_cb)

        self.createGraph()

    def availableElevatorFor(self, msg, index):
        if msg.data == self.robot_id:
            self.elevator_available[index] = msg.data

    def createGraph(self):
        self.graph = nx.Graph()
        self.graph.add_node("start")

        for n in self.manager.maps:
            rospy.loginfo("Adding node !"+n)
            self.graph.add_node(n)

        self.graph.add_node("end")
        nx.set_node_attributes(self.graph,"x", 0)
        nx.set_node_attributes(self.graph,"y", 0)

    def plotGraph(self):
        pos = nx.spring_layout(self.graph)
        nx.draw(self.graph, pos, with_labels=True)
        #y_node_labels = nx.get_node_attributes(self.graph,'x')
        #x_node_labels = nx.get_node_attributes(self.graph,'y')
        #edge_labels=nx.draw_networkx_edge_labels(self.graph,pos=nx.spring_layout(G))
        #nx.draw_networkx_labels(self.graph, pos, labels = edge_labels)
        plt.show()

    def resetGraph(self):
        rospy.loginfo("Reseting Graph")
        self.graph.remove_node("start")
        self.graph.remove_node("end")

    def cancel_cb(self, msg):
        self.preempt_goal = msg.data

    def execute_cb(self, goal):
        #print goal.goal_map
        #Node of current_map must match with current pose
        robot_pos = self.manager.get_robot_position()

        #The cost of moving through a wormhole is nothing. In the future, this
        #could be non-zero if there is an object such as a door or elevator that
        #could slow the robot's motion.

        #Create the graph for each of the wormholes
        self.resetGraph()
        #self.plotGraph()

        self.graph.add_edge("start",self.manager.current_map,weight = 0)

        for w in self.manager.wormholes:
            for l in w["locations"]:
                node_pose = [self.graph.node[w["name"]]['x'],  self.graph.node[w["name"]]['y']]
                self.graph.add_edge(w["name"],l["map"], weight = calc_distance(node_pose, l["position"]))

        self.graph.node[self.manager.current_map]['x'] = robot_pos[0]
        self.graph.node[self.manager.current_map]['y'] = robot_pos[1]

        #Node of end goal must match with goal pose
        self.graph.node[goal.goal_map]['x'] = goal.target_pose.pose.position.x
        self.graph.node[goal.goal_map]['y'] = goal.target_pose.pose.position.y

        self.graph.add_edge(goal.goal_map,"end",weight = 0)
        rospy.loginfo ("Looking for path from  %s to %s", self.manager.current_map , goal.goal_map)

        try:
            #Find Shortest Path
            path = nx.astar_path(self.graph, "start", "end")
        except:
            rospy.logerr("Path NOT FOUND")
            return None

        rospy.loginfo ("PATH FOUND")
        print "The found path is ", path

        #removing start and end
        path = path[1:-1]

        offset = []
        old_north = 0.0

        old_pos = [None, None]
        old_angle = None

        while (goal.goal_map != self.manager.current_map):

            #Look For Wormhole
            wormhole = None

            #Find Wormhole
            for i in self.manager.wormholes:
                if (i["name"] == self.manager.current_map):
                    wormhole = i

            #If wormhole not found throw error
            if wormhole is None:
                rospy.logerr("Wormhole " + self.manager.current_map + " Not Defined")
                self.action_server.set_aborted(MultiMapNavigationResult())
                return None

            #This should not happened but might handle unexpected errors
            if not (len(path) > 1):
                rospy.logerr("Path can not be length 0")
                self.action_server.set_aborted(MultiMapNavigationResult())
                return None

            location = None

            for w in wormhole["locations"]:
              #Querrying Next Map
              if path[1] is w["map"]:
                  location = w

            if location is None:
                rospy.logerr("Location  " + path[1] + " Not Defined n Wormhole")
                return None

            pos = location["position"]
            #
            #
            # position_pose = PoseStamped()
            # position_pose.header.frame_id = "map"
            # position_pose.pose.position.x = pos[0]
            # position_pose.pose.position.y = pos[1]
            # position_pose.pose.orientation.w = 1
            # self.current_goal_pub.publish(position_pose)
            # rospy.loginfo("Send Position To Move_Base " )

            current_map = path[0]

            #Other Transisitons
            # wormhole_goal = None
            #
            # wormhole_type = wormhole["type"]
            # wormhole_goal = MultiMapNavigationTransitionGoal()
            # wormhole_goal.wormhole = yaml.dump(wormhole)
            # wormhole_goal.start = path[0].split("_")[0]
            # wormhole_goal.end = path[1].split("_")[0]

            angle = 0
            radius = None

            offset = self.manager.get_robot_position()
            offset[0] = offset[0] - pos[0]
            offset[1] = offset[1] - pos[1]

            #in case of elevator wait in waiting aree until elevator is available then go to Location
            self.drivingToWormhole(location, wormhole)
            #self.manager.current_map = mapname
            #rospy.loginfo("Skipped move base because the goal location is the current location")

            rospy.loginfo("Going to" + path[1])
            custom_goal = MultiMapServerGoal()
            custom_goal.map_name = path[1]
            cli = self.manager.transition_action_clients["custom"]
            cli.send_goal(custom_goal)
            cli.wait_for_result()
            self.manager.current_map = path[1]


            self.afterSwitchingMap(current_map, location, wormhole, offset)

            #Remove First Node of Path
            path = path[1:]
            current_map = path[0]

        #Get to the end pointf
        rospy.loginfo("In final Map")
        msg = MoveBaseGoal()
        msg.target_pose = goal.target_pose

        wasGoalSuccessful = self.go_to_goal(msg, None)
        if not wasGoalSuccessful:
            rospy.loginfo("Goal aborted!")
            self.action_server.set_aborted(MultiMapNavigationResult())
            return None

        self.action_server.set_succeeded(MultiMapNavigationResult())

    def wait_for_elevator(self, target_floor, elevator_id):
        print "elevator_id", elevator_id
        elevator_target = multi_map_navigation.msg.MultiMapNavigationTargetElevatorGoal()
        elevator_target.elevatorTargetFloor = target_floor
        elevator_target.elevatorId = str(elevator_id)

        client = self.manager.transition_action_clients["elevator_blast"]
        client.send_goal_and_wait(elevator_target, execute_timeout=rospy.Duration.from_sec(600.0),  preempt_timeout=rospy.Duration.from_sec(700.0)) # maximum wait time for elevator: ~10 minutes

    def go_to(self, waiting_point, place_name):

        rospy.loginfo("But heading to %s", place_name)

        pos_str = waiting_point[0]
        waiting_pose = [float(x) for x in pos_str.split()]

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
                rospy.sleep(0.5)
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
            rospy.loginfo("Position Reached")
        return True

    def afterSwitchingMap(self,mapname, location, wormhole, offset):
        #Create and publish the new pose for AMCL
        print "Switching Maps"
        l = tf.TransformListener()
        l.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(4.0))
        print "Transform received"
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.get_rostime()
        pos = location["position"]

        offset_angle = 0.0
        offset_radius = 0.0

        angle = 0
        if "radius" in wormhole:
            offset_angle = math.atan2(offset[1], offset[0])
            offset_radius = math.sqrt(offset[0] * offset[0] + offset[1] * offset[1])
            offset_angle += 0 #north - old_north

        if "angle" in location:
            angle = location["angle"]

        msg.pose.pose.position.x = pos[0] #+ math.cos(offset_angle) * offset_radius
        msg.pose.pose.position.y = pos[1] #+ math.sin(offset_angle) * offset_radius
        msg.pose.pose.position.z = 0.0

        roll, pitch, yaw = tf.transformations.euler_from_quaternion( \
        self.manager.get_robot_rotation())
        #print yaw
        yaw += 0 #north - old_north
        if (angle):
            yaw = angle
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        self.pose_pub.publish(msg)

        rospy.sleep(3) #FIXME: come up with an alternative approach to know when AMCL is done

        #NOT SO SURE HOW IT WORKS
        rospy.loginfo("Wait for movebase")
        self.move_base.wait_for_server()
        rospy.loginfo("Done")

    def drivingToWormhole(self, location, wormhole):
        #We need to do driving to get to the next wormhole
        #print pos
        #Create the goal for the next waypoint (at the target)
        rospy.loginfo("Heading towards a wormhole")

        #Waiting Point is inside the elevator
        if (wormhole["type"] == "elevator_blast"):
            elevator_request = elevatorRequest()
            elevator_request.header.stamp = rospy.Time.now()
            elevator_request.robot_id = self.robot_id
            elevator_requested_floor = location["floor"]
            self.elevator_request_pub.publish(elevator_request)

            wasGoalSuccessful = self.go_to(location["waiting_point"], "waiting_point")

            if not wasGoalSuccessful:
                rospy.loginfo("Goal aborted!")
                return None;

            #GO TO ELEVATOR
            self.check_elevator_availability(location["elevator_id"])
        else:
            rospy.loginfo("Wormhole Type" + wormhole["type"] + " detected")
        angle = 0
        radius = 0

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

        pos = location["position"]
        msg = MoveBaseGoal()
        msg.target_pose.header.stamp = rospy.get_rostime()
        msg.target_pose.header.frame_id = "/map"
        msg.target_pose.pose.position.x = pos[0]
        msg.target_pose.pose.position.y = pos[1]
        msg.target_pose.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, angle)
        msg.target_pose.pose.orientation.w = quat[3]
        msg.target_pose.pose.orientation.x = quat[0]
        msg.target_pose.pose.orientation.y = quat[1]
        msg.target_pose.pose.orientation.z = quat[2]

        wasGoalSuccessful = self.go_to_goal(msg, radius)
        if not wasGoalSuccessful:
            rospy.loginfo("Goal aborted!")
            return None;
        else:
            rospy.loginfo("Position Reacheds")

        if (wormhole["type"] == "elevator_blast"):
            self.wait_for_elevator(location["floor"], location["elevator_id"]) # call elevator to current floor

        rospy.loginfo("Done move_base")

    def check_elevator_availability(self, elevator_id):
        rospy.loginfo("Wait For Elevator %s", str(elevator_id))
        while not self.elevator_available[elevator_id]:
            pass
