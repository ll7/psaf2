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


class LaneletMapProviderTest:
    """
    Check if the right thing was published by the lanelet_map_provider.
    Wirtes two files in .ros: "published_test_map.osm" and "org_test_map.osm".
    If files are the same excpet for the ids the everything worked fine.
    You can check the results graphically in josm-editor.
    """
    def __init__(self, role_name):
        self.role_name = role_name
        self.lanelet_sub = rospy.Subscriber("/psaf/lanelet_map", LaneletMap, self.map_recieved)
        self.osm_sub = rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.load_opendrive)

        self.org_lanelet_map = None
        self.published_lanelet_map = None

    def map_recieved(self, lanelet_msg):
        print("Published Map recieved")
        self.published_lanelet_map = lanelet2.io.load(lanelet_msg.lanelet_bin_path)

        self.calc_route_cost(self.published_lanelet_map, "./published_test_map.osm")

        time.sleep(5)
        if self.published_lanelet_map == self.org_lanelet_map:
            print("They are the same")
        else:
            print("They are not the same")
            print(self.published_lanelet_map.__dict__)
            # print(self.org_lanelet_map.__dict__)


    def load_opendrive(self, msg):
        print("Load OpenDrive Map")
        # parse the byte encoded xml opendrive tree
        opendrive_byte = io.BytesIO(msg.opendrive.encode())
        opendrive = parse_opendrive(etree.parse(opendrive_byte).getroot())

        # convert opendrive to commonroad scenario
        scenario = self.convert_opendrive(opendrive)

        # compute openstreetmap string representation of the commonroad scenario with right projection
        l2osm = L2OSMConverter("+proj=omerc +lat_0=49 +lonc=8 +alpha=0 +k=1 +x_0=0 +y_0=0 +gamma=0 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0")
        openstreetmap = etree.tostring(l2osm(scenario), xml_declaration=True, encoding="UTF-8", pretty_print=True)

        # save osm map to later read with lanlet2
        osm_out_path = "./osm_test.osm"
        with open(osm_out_path ,"wb+") as mapfile:
            mapfile.write(openstreetmap)  
        
        # parse to lanelet2
        projection = lanelet2.projection.UtmProjector(lanelet2.io.Origin(49, 8))
        lanelet_map = lanelet2.io.load(osm_out_path, projection)

        self.calc_route_cost(lanelet_map, "./org_test_map.osm")

        self.org_lanelet_map = lanelet_map

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


    def convert_opendrive(self, opendrive: OpenDrive) -> Scenario:
        """Convert an existing OpenDrive object to a CommonRoad Scenario.

        Args:
          opendrive: Parsed in OpenDrive map.
        Returns:
          A commonroad scenario with the map represented by lanelets.
        """
        road_network = Network()
        road_network.load_opendrive(opendrive)

        return road_network.export_commonroad_scenario()

    def make_positive(self, layer):
        """
        Make the IDs of the layer positive (default is negative).
        """
        for elem in layer:
            if elem.id < 0:
                elem.id = layer.uniqueId()

    
if __name__ == "__main__":
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    lmp = LaneletMapProviderTest(role_name)
    
    # idle so topic is still present
    # (resp. if world changes)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

