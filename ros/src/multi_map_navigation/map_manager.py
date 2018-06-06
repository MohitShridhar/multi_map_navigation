import rospy
import tf
import actionlib
from multi_map_navigation.msg import *
from multi_map_navigation.srv import *
from std_msgs.msg import String
from visualization_msgs.msg import *
import yaml

class MultiMapManager(object):
    def __init__(self):
        self.ready = False
        self.setup_namespaces()
        self.listener = tf.TransformListener()
        self.list_maps = ["kitchen", "living-room",'dining-room','shelf']
        self.current_map_name_pub = rospy.Publisher("multi_map_server/map_name", String, queue_size=1)
        self.wormhole_marker_pub = rospy.Publisher('wormhole_marker', MarkerArray, queue_size=1)
        self.waiting_marker_pub = rospy.Publisher('waiting_point', MarkerArray, queue_size=1)

        self.n_markers = 0

        self.transition_action_clients = {}
        transitions = ["door_blast", "elevator_blast", "door_drag"]

        if rospy.has_param('~transition_types'):
            transitions = rospy.get_param("~transition_types").split(" ")


        for client in transitions:
            if (client.strip() == ""):
                continue
            rospy.loginfo("Waiting for " + client)

            if (client.strip() == "custom"):
                rospy.loginfo("Adding Custom with MultiMapServerAction")
                cli = actionlib.SimpleActionClient(client, MultiMapServerAction)
            elif (client.strip() == "elevator_blast"):
                cli = actionlib.SimpleActionClient(client, MultiMapNavigationTargetElevatorAction)
            else:
                cli = actionlib.SimpleActionClient(client, MultiMapNavigationTransitionAction)
                #cli.wait_for_server()
            self.transition_action_clients[client] = cli

        self.definition_file = None

        if rospy.has_param('~definition_file'):
            self.definition_file = rospy.get_param("~definition_file")
        else:
            rospy.logerr("You must specify a definition_file")
            return

        if (not self.loadyaml(self.definition_file)):
            return

        rospy.loginfo("Waiting for position")
        rospy.sleep(3)
        self.get_robot_position()
        self.ready = True

        rospy.loginfo("Starting")

    def setup_namespaces(self):

        self.robot_namespace = "" # default namespace (not recommended)
        self.base_frame = "base_link"
        if rospy.has_param('~base_frame'):
            self.base_frame = rospy.get_param("~base_frame")

        self.base_frame = "/" + self.base_frame
        rospy.loginfo("base_frame set as " + self.base_frame)

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
        #rospy.sleep(2)
        marker_array = MarkerArray()
        waiting_area_marker_array = MarkerArray()
        elevator_type = False

        for i in self.wormholes:
            if i["name"] == self.current_map:
                if i["type"] == "elevator_blast":
                    elevator_type = True

                for j in i["locations"]:
                    wormhole_marker = Marker()
                    wormhole_marker.header.frame_id = "map"
                    wormhole_marker.header.stamp = rospy.get_rostime()
                    wormhole_marker.ns = "multimna"
                    wormhole_marker.type = Marker.CYLINDER
                    wormhole_marker.action = Marker.MODIFY
                    wormhole_marker.id = n_markers
                    wormhole_marker.pose.position.x = j["position"][0]
                    wormhole_marker.pose.position.y = j["position"][1]
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

                    if elevator_type:
                        waiting_pose = [ float(i) for i in j["waiting_point"][0].split()]
                        waiting_area_marker = Marker()
                        waiting_area_marker.header.frame_id = self.robot_namespace + "/map"
                        waiting_area_marker.header.stamp = rospy.get_rostime()
                        waiting_area_marker.ns = "multimna"
                        waiting_area_marker.type = Marker.CYLINDER
                        waiting_area_marker.action = Marker.MODIFY
                        waiting_area_marker.id = n_markers
                        waiting_area_marker.pose.position.x = waiting_pose[0]
                        waiting_area_marker.pose.position.y = waiting_pose[1]
                        waiting_area_marker.pose.position.z = 0.25
                        waiting_area_marker.pose.orientation.x = 0.0
                        waiting_area_marker.pose.orientation.y = 0.0
                        waiting_area_marker.pose.orientation.z = 0.0
                        waiting_area_marker.pose.orientation.w = 1.0
                        waiting_area_marker.scale.x = 0.15 * 2.0 # fixed radius size for wait_area goals
                        waiting_area_marker.scale.y = 0.15 * 2.0
                        waiting_area_marker.scale.z = 1
                        waiting_area_marker.color.a = 0.7
                        waiting_area_marker.color.r = 1.0
                        waiting_area_marker.color.g = 0.0
                        waiting_area_marker.color.b = 0.0
                        waiting_area_marker_array.markers.append(waiting_area_marker)

        if elevator_type:
            self.waiting_marker_pub.publish(waiting_area_marker_array)

        if len(marker_array.markers) > 0:
            self.wormhole_marker_pub.publish(marker_array)


        if (self.n_markers > n_markers):
            for i in range(n_markers, self.n_markers):
                wormhole_marker = Marker()
                wormhole_marker.action = Marker.DELETE
                wormhole_marker.id = self.n_markers

                waiting_area_marker = Marker()
                waiting_area_marker = Marker.DELETE
                #waiting_area_marker.id = self.n_markers

                self.wormhole_marker_pub.publish(wormhole_marker)
        self.n_markers = n_markers

    def loadyaml(self, filename):
        try:
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
            if (not "name" in i):
                rospy.logerr("YAML file: " + filename + " contains an invalid map with no name")
                return False
            print "maps available", i["name"]
            self.maps[i["name"]] =  [i["name"]]

        self.wormholes = data["wormholes"]
        n = 0
        wh_names = []
        for i in self.wormholes:
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
