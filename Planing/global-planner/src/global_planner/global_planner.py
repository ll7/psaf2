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
from custom_carla_msgs.msg import GlobalPath
from geometry_msgs.msg import Point
from pathlib import Path
from sensor_msgs.msg import NavSatFix
from collections import deque


class GlobalPlanner:
    """
    Check if the right thing was published by the lanelet_map_provider.
    Wirtes two files in .ros: "published_test_map.osm" and "org_test_map.osm".
    If files are the same excpet for the ids the everything worked fine.
    You can check the results graphically in josm-editor.
    """

    def __init__(self, role_name):
        self.role_name = role_name
        self.lanelet_sub = rospy.Subscriber("/psaf/lanelet_map", LaneletMap, self.map_received)
        self.target_sub = rospy.Subscriber("/psaf/target_point", NavSatFix, self.target_received)
        self.gnss_subscriber = rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, self.gnss_received)
        self.path_pub = rospy.Publisher("/psaf/global_path", GlobalPath, queue=1, latch=True)

        self.projection = lanelet2.projection.UtmProjector(lanelet2.io.Origin(49, 8))
        self.route = deque()

        self.lanelet_map = None
        self.start_gnss = None
        self.target_gnss = None

    def gnss_received(self, gnss):
        self.start_gnss = gnss

    def target_received(self, target_point):
        self.target_gnss = target_point
        while self.start_gnss is None or self.lanelet_map is None:
            rospy.sleep(0.05)

        self.create_global_plan()

    def map_received(self, lanelet_msg):
        self.lanelet_map = lanelet2.io.load(lanelet_msg.lanelet_bin_path, self.projection)
        # lanelet2.io.write("./lanelet_map.osm", self.published_lanelet_map, self.projection)

    def find_nearest_lanelet(self, gnss_point):
        cartesian_pos = self.projection.forward(
            lanelet2.core.GPSPoint(gnss_point.latitude + 49, gnss_point.longitude + 8, gnss_point.altitude))
        basic_point = lanelet2.core.BasicPoint2d(cartesian_pos.x, -cartesian_pos.y)
        return lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, basic_point, 1)[0][1]

    def create_global_plan(self):
        # Crate Routiing Graph
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)
        graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules)

        # Get nearest Lanelet to start/target gps point
        start_lanelet = self.find_nearest_lanelet(self.start_gnss)
        target_lanelet = self.find_nearest_lanelet(self.target_gnss)

        # Get all possible routes
        route = graph.getRoute(start_lanelet, target_lanelet)
        shortest_route = route.shortestPath()

        # for llet in shortest_route.getRemainingLane(start_lanelet):
        #     self.lanelet_map.laneletLayer[llet.id].attributes["shortestPath"] = "True"
        # sp_path = "./_shortestpath.osm"
        # lanelet2.io.write(sp_path, self.lanelet_map, self.projection)

        for lane in shortest_route.getRemainingLane(start_lanelet):
            for p in lane.centerline:
                point = Point()
                point.x = p.x
                point.y = -p.y
                point.z = p.z

                self.route.append(point)

        path_msg = GlobalPath()
        path_msg.path = self.route
        print(self.route)
        self.path_pub.publish(path_msg)


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
