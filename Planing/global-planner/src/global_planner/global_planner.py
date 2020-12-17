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
from pathlib import Path
from sensor_msgs.msg import NavSatFix


class GlobalPlanner:
    """
    Check if the right thing was published by the lanelet_map_provider.
    Wirtes two files in .ros: "published_test_map.osm" and "org_test_map.osm".
    If files are the same excpet for the ids the everything worked fine.
    You can check the results graphically in josm-editor.
    """
    def __init__(self, role_name):
        self.role_name = role_name
        self.lanelet_sub = rospy.Subscriber("/psaf/lanelet_map", LaneletMap, self.map_recieved)
        self.projection = lanelet2.projection.UtmProjector(lanelet2.io.Origin(49, 8))
        self.gnss = NavSatFix()
        self.gnss_subscriber = rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, self.gnss_received)


    def gnss_received(self, gnss):
        self.gnss = gnss



    def map_recieved(self, lanelet_msg):
        self.published_lanelet_map = lanelet2.io.load(lanelet_msg.lanelet_bin_path, self.projection)
        lanelet2.io.write("./lanelet_map.osm", self.published_lanelet_map, self.projection)
        self.create_global_plan(self.published_lanelet_map)


    def create_global_plan(self, lmap):
        # Crate Routiing Graph
        trafficRules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        graph = lanelet2.routing.RoutingGraph(lmap, trafficRules)

        # Get nearest Lanelet to start/target gps point
        carthesian_pos = self.projection.forward(lanelet2.core.GPSPoint(self.gnss.latitude + 49, self.gnss.longitude + 8, self.gnss.altitude))
        pstart = (carthesian_pos.x, -carthesian_pos.y)
        start_point = lanelet2.core.BasicPoint2d(pstart[0], pstart[1])
        target_point = lanelet2.core.BasicPoint2d(160, 185)
        start_lanelet = lanelet2.geometry.findNearest(lmap.laneletLayer, start_point, 1)[0][1]
        target_lanelet = lanelet2.geometry.findNearest(lmap.laneletLayer, target_point, 1)[0][1]

        # Get all possible routes
        route = graph.getRoute(start_lanelet, target_lanelet)
        shortest_route = route.shortestPath()

        for llet in shortest_route.getRemainingLane(start_lanelet):
            lmap.laneletLayer[llet.id].attributes["shortestPath"] = "True"
        sp_path = "./_shortestpath.osm"
        lanelet2.io.write(sp_path, lmap, self.projection)
        print("_______________________________________________________________________________________________________________________________")


    
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

