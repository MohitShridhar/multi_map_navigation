import rospy
import math
import random
import tf
import networkx as nx
import actionlib
import actionlib_msgs
from std_msgs.msg import Bool

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
        if rospy.has_param('~move_base_action'):
            move_base_name = rospy.get_param("~move_base_action")
        self.move_base = actionlib.SimpleActionClient(move_base_name, MoveBaseAction)
        self.move_base.wait_for_server()
        self.action_server = actionlib.SimpleActionServer("multi_map_navigation/move", MultiMapNavigationAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
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
        graph = nx.Graph()

        for n in self.manager.maps:
            graph.add_node(n)

        nx.set_node_attributes(graph,"x", 0)
        nx.set_node_attributes(graph,"y", 0)
        robot_pos = self.manager.get_robot_position()

        #Node of current_map must match with current pose
        graph.node[self.manager.current_map]['x'] = robot_pos[0]
        graph.node[self.manager.current_map]['y'] = robot_pos[1]

        #Node of end goal must match with goal pose
        graph.node[goal.goal_map]['x'] = goal.target_pose.pose.position.x
        graph.node[goal.goal_map]['y'] = goal.target_pose.pose.position.y


        #The cost of moving through a wormhole is nothing. In the future, this
        #could be non-zero if there is an object such as a door or elevator that
        #could slow the robot's motion.

        #Create the graph for each of the wormholes
        for w in self.manager.wormholes:
            for l in w["locations"]:
                node_pose = [graph.node[w["name"]]['x'],  graph.node[w["name"]]['y']]
                graph.add_edge(w["name"],l["map"], weight = calc_distance(node_pose, l["position"]))

        print graph.nodes()

        path = nx.astar_path(graph, self.manager.current_map, goal.goal_map)
        print "PATH FOUND", path , " from " , self.manager.current_map , " to " , goal.goal_map

        offset = []
        old_north = 0.0

        old_pos = [None, None]
        old_angle = None

        while (path[0] != self.manager.current_map):
            #wormhole
            name = path[0][path[0].find("_") + 1:]
            print name
            wormhole = None
            for i in self.manager.wormholes:
                if (i["name"] == name):
                    wormhole = i
            #print wormhole
            print wormhole["locations"]
            location = wormhole["locations"][name]
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
                self.manager.current_map = mapname
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
                #self.pose_pub.publish(msg)
                #Select the new map
                #self.manager.select_map(mapname)
	            #self.manager.current_map_name_pub.publish(mapname)
                emptySrv = Empty()
                #rospy.wait_for_service("/global_localization")

                try:
                    init_amcl = rospy.ServiceProxy("/global_localization", Empty)
                    init_amcl()
                    pass
                except:
                    rospy.logerr("Could not re-initialize AMCL")

                rospy.loginfo("%s pretend to publsh ", msg.pose.pose)
                self.pose_pub.publish(msg)

                #rospy.sleep(2) #FIXME: come up with an alternative approach to know when AMCL is done


                #NOT SO SURE HOW IT WORKS
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

                if (wormhole["type"] == "elevator_blast"):
                    wasGoalSuccessful = self.go_to_waiting_point(location["waiting_point"])
                    if not wasGoalSuccessful:
                        rospy.loginfo("Goal aborted!")
                        return None;

                    self.target_elevator(location["floor"], wormhole["name"]) # call elevator to current floor

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
            print "queue " , queue
            print "distances ", distances
            for v in queue:
                if (vertex is None):
                    vertex = v
                if (queue[v] < queue[vertex]):
                    vertex = v
            #Dijksta
            print "vertex " , vertex
            distances[vertex] = queue[vertex]
            print "distances", distances[vertex]

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
