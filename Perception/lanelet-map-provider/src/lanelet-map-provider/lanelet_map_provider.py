import os
import sys
import io
import rospy
import lanelet2

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


class LaneletMapProvider:
    """
    Class to subscribe /carla/WorldInfo topic and create /psaf/LaneletMap topic.
    """
    
    def __init__(self, role_name):
        self.role_name = role_name
        self.sub = rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.convert_open_drive_map)
        self.lanelet_publisher = rospy.Publisher("/psaf/lanelet_map", LaneletMap, queue_size=1, latch=True)
        self.lanelet_msg = LaneletMap()
        self.lanelet_msg.lanelet_bin_path = str(Path.home()) + "/lanelet_bin.bin"
 
    def convert_open_drive_map(self, msg):  
        """
        Read opendrive string from message and save as lanelet binary file.
        Then publish path to file.
        """
        # parse the byte encoded xml opendrive tree
        #bytes_msg = bytes(msg.opendrive, 'utf-8')
        opendrive_byte = io.BytesIO(msg.opendrive.encode())
        opendrive = parse_opendrive(etree.parse(opendrive_byte).getroot())

        # convert opendrive to commonroad scenario
        scenario = self.convert_opendrive(opendrive)

        # compute openstreetmap string representation of the commonroad scenario with right projection
        l2osm = L2OSMConverter("+proj=omerc +lat_0=49 +lonc=8 +alpha=0 +k=1 +x_0=0 +y_0=0 +gamma=0 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0")
        openstreetmap = etree.tostring(l2osm(scenario), xml_declaration=True, encoding="UTF-8", pretty_print=True)

        # save osm map to later read with lanlet2
        osm_out_path = "./osm.osm"
        with open(osm_out_path ,"wb+") as mapfile:
            mapfile.write(openstreetmap)  
        
        # parse to lanelet2
        projection = lanelet2.projection.UtmProjector(lanelet2.io.Origin(49, 8))
        lanelet_map = lanelet2.io.load(osm_out_path, projection)
        
        # with open(lanelet_bin_path, "w+") as binfile:
        lanelet2.io.write(self.lanelet_msg.lanelet_bin_path , lanelet_map, projection)
                       
        # publish path to binary data
        self.lanelet_publisher.publish(self.lanelet_msg)
            
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

    lmp = LaneletMapProvider(role_name)
    
    # idle so topic is still present
    # (resp. if world changes)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



