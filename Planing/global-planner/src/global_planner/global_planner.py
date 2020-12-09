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


    def map_recieved(self, lanelet_msg):
        self.published_lanelet_map = lanelet2.io.load(lanelet_msg.lanelet_bin_path)

        self.create_global_plan(self.published_lanelet_map)


    def create_global_plan(self, lmap):
        trafficRules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        graph = lanelet2.routing.RoutingGraph(lmap, trafficRules)
        print("Creation Started")

        lanelets = lmap.laneletLayer
        for elem in lanelets
            print(elem)

        startLane = lmap.laneletLayer[2797] # lanelet IDs
        endLane = lmap.laneletLayer[2938]
        pint("Lanelets found")
        if rt is None:
            print("error: no route was calculated")
        else:
            sp = rt.shortestPath()
            if sp is None:
                print ("error: no shortest path was calculated")
            else:
                print ([l.id for l in sp.getRemainingLane(startLane)])

        # save the path in another OSM map with a special tag to highlight it
        if sp:
            for llet in sp.getRemainingLane(startLane):
                lmap.laneletLayer[llet.id].attributes["shortestPath"] = "True"
            projector = lanelet2.projection.MercatorProjector(lorigin)
            sp_path = "./_shortestpath.osm"
            lanelet2.io.write(sp_path, lmap, projector)
        print("Creation Done")

    def calc_route_cost(self, lanelet_map, out_path):
        self.make_positive(lanelet_map.pointLayer)
        self.make_positive(lanelet_map.lineStringLayer)
        self.make_positive(lanelet_map.polygonLayer)
        self.make_positive(lanelet_map.laneletLayer)
        self.make_positive(lanelet_map.areaLayer)
        self.make_positive(lanelet_map.regulatoryElementLayer)

        rules_map = {"vehicle": lanelet2.traffic_rules.Participants.Vehicle,
                     "bicycle": lanelet2.traffic_rules.Participants.Bicycle,
                     "pedestrian": lanelet2.traffic_rules.Participants.Pedestrian,
                     "train": lanelet2.traffic_rules.Participants.Train}
        proj = lanelet2.projection.UtmProjector(lanelet2.io.Origin(49, 8))


        routing_cost = lanelet2.routing.RoutingCostDistance(0.)  # zero cost for lane changes
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      rules_map["vehicle"])
        graph = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules, [routing_cost])
        debug_map = graph.getDebugLaneletMap()

        lanelet2.io.write(out_path, debug_map, proj)

    
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

