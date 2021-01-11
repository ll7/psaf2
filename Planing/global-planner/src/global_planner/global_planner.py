import os
import sys
import io
import rospy
import lanelet2
import time
from lxml import etree
from lanelet2.projection import UtmProjector
from commonroad.scenario.scenario import Scenario
from opendrive2lanelet.opendriveparser.elements.opendrive import OpenDrive
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.network import Network
from opendrive2lanelet.osm.lanelet2osm import L2OSMConverter
from carla_msgs.msg import CarlaWorldInfo
from custom_carla_msgs.msg import LaneletMap
# from custom_carla_msgs.msg import GlobalPath
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from pathlib import Path
from sensor_msgs.msg import NavSatFix
from collections import deque
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
import math


class GlobalPlanner:
    """
    Check if the right thing was published by the lanelet_map_provider.
    Wirtes two files in .ros: "published_test_map.osm" and "org_test_map.osm".
    If files are the same excpet for the ids the everything worked fine.
    You can check the results graphically in josm-editor.
    """

    def __init__(self, role_name):
        self.role_name = role_name

        self.projection = lanelet2.projection.UtmProjector(lanelet2.io.Origin(0, 0))
        self.route = deque()

        self.lanelet_map = None
        self.start_gnss = None
        self.target_gnss = None
        self.odometry = None

        self.position = None
        self.target = None

        self.graph = None


        self.lanelet_sub = rospy.Subscriber("/psaf/lanelet_map", LaneletMap, self.map_received)
        self.target_sub = rospy.Subscriber("/psaf/target_point", NavSatFix, self.target_received)
        # self.path_pub = rospy.Publisher("/psaf/global_path", GlobalPath, queue_size=1, latch=True)
        self.gnss_subscriber = rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, self.gnss_received)
        self.path_pub = rospy.Publisher("/psaf/global_path", Path, queue_size=1, latch=True)
        self.odometry_sub = rospy.Subscriber("carla/ego_vehicle/odometry", Odometry, self.odometry_received)
        self.inital_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_received)



    def init_pose_received(self, pose):
        print("New Start")
        while self.start_gnss is None or self.lanelet_map is None or self.odometry is None:
            rospy.sleep(0.05)

        self.create_global_plan()

    def gnss_received(self, gnss):
        self.start_gnss = gnss

    def odometry_received(self, odometry):
        self.odometry = odometry

    def target_received(self, target_point):
        print("Target Received")
        self.target_gnss = target_point
        #while self.start_gnss is None or self.lanelet_map is None or self.odometry is None:
        #    rospy.sleep(0.05)

        #self.create_global_plan()

    def map_received(self, lanelet_msg):
        self.lanelet_map = lanelet2.io.load(lanelet_msg.lanelet_bin_path, self.projection)
        # lanelet2.io.write("./lanelet_map.osm", self.published_lanelet_map, self.projection)

    def find_nearest_lanelet(self, gnss_point):
        cartesian_pos = self.projection.forward(
            lanelet2.core.GPSPoint(gnss_point.latitude, gnss_point.longitude, gnss_point.altitude))
        basic_point = lanelet2.core.BasicPoint2d(cartesian_pos.x, cartesian_pos.y)
        return (lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, basic_point, 1)[0][1], cartesian_pos)

    def create_global_plan(self):
        print("Plan Creation Stated!")
        
        if self.graph is None:
            # Crate Routiing Graph
            traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
            routing_cost = lanelet2.routing.RoutingCostDistance(0.)
            self.graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules, [routing_cost])
            
            # self.debug_graph = self.graph.getDebugLaneletMap()
            # lanelet2.io.write("./debug_lanlet2_map.osm", self.debug_graph)
        
        start_lanelet, self.position = self.find_nearest_lanelet(self.start_gnss)
        # Get nearest Lanelet to start/target gps point
        target_lanelet, self.target = self.find_nearest_lanelet(self.target_gnss)

        # Get all possible routes
        route = self.graph.getRoute(start_lanelet, target_lanelet, 0, True)
        shortest_route = route.shortestPath()

        # for llet in shortest_route.getRemainingLane(start_lanelet):
        #     self.lanelet_map.laneletLayer[llet.id].attributes["shortestPath"] = "True"
        # sp_path = "./_shortestpath.osm"
        # lanelet2.io.write(sp_path, self.lanelet_map, self.projection)



        for lane in shortest_route.getRemainingLane(start_lanelet):
            for p in lane.centerline:
                point = Point()
                point.x = p.x
                point.y = p.y
                point.z = p.z
                self.route.append(point)

        print("--\n--\n--\n--\n--\n--\n--\n--\n--\n--\n--\n")
        print(len(self.route))
        
        self.sanitize_route()

        print(len(self.route))
        print("--\n--\n--\n--\n--\n--\n--\n--\n--\n--\n--\n")

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for p in self.route:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = p.x
            pose.pose.position.y = p.y
            pose.pose.position.z = p.z
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def sanitize_route(self):
        # delete unnecessary points at begin and end of route
        max_index = -1
        min_index = -1
        min_distance_start = None
        min_distance_end = None

        # find last point before start position
        for i, point in enumerate(self.route):
            distance, _ = self.compute_magnitude_angle(point, self.position, self.odometry.pose.pose.orientation)
            
            if min_distance_start is not None:
                if distance < min_distance_start:
                    max_index = i
                    min_distance_start = distance
            else:
                min_distance_start = distance

        # delete points before start position
        if max_index >= 0:
            for i in range(max_index + 1):
                self.route.popleft()

        # # find last point after target position
        # for i, point in enumerate(reversed(self.route)):
        #     distance, _ = self.compute_magnitude_angle(point, self.target, self.odometry.pose.pose.orientation)
            
        #     if min_distance_end is not None:
        #         if distance < min_distance_end:
        #             min_index = i
        #             min_distance_end = distance
        #     else:
        #         min_distance_end = distance

        # # delete points after target position
        # if min_index >= 0:
        #     for i in range(min_index + 1):
        #         self.route.pop()

    def compute_magnitude_angle(self, target_location, current_location, qorientation):
        """
        Compute relative angle and distance between a target_location and a current_location
        """
        # angle of vehicle
        quaternion = (
            qorientation.x,
            qorientation.y,
            qorientation.z,
            qorientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        orientation = math.degrees(-yaw)

        # vector from vehicle to target point and distance
        target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
        norm_target = np.linalg.norm(target_vector)

        # vector of the car and absolut angle between vehicle and target point
        forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
        d_angle = math.degrees(math.acos(np.dot(forward_vector, target_vector) / norm_target))

        # make angle negative or positive
        cross = np.cross(forward_vector, target_vector)
        if cross > 0:
            d_angle *= -1.0

        return (norm_target, d_angle)



if __name__ == "__main__":
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    gp = GlobalPlanner(role_name)

    # idle so topic is still present
    # (resp. if world changes)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
