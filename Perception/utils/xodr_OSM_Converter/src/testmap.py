import os
import sys

from lxml import etree
from io import BytesIO
import lanelet2
from lanelet2.projection import UtmProjector
import argparse
from commonroad.scenario.scenario import Scenario
from opendrive2lanelet.opendriveparser.elements.opendrive import OpenDrive
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.network import Network
from opendrive2lanelet.osm.lanelet2osm import L2OSMConverter
# from xodr.src.Converter import Converter
from Converter import Converter

def make_positive(layer):
    """
    Make the IDs of the layer positive (default is negative).
    """
    for elem in layer:
        if elem.id < 0:
            elem.id = layer.uniqueId()
            
            
def convert_opendrive(opendrive: OpenDrive) -> Scenario:
  """Convert an existing OpenDrive object to a CommonRoad Scenario.

  Args:
    opendrive: Parsed in OpenDrive map.
  Returns:
    A commonroad scenario with the map represented by lanelets.
  """
  road_network = Network()
  road_network.load_opendrive(opendrive)

  return road_network.export_commonroad_scenario()



def main():

    map_path = "/home/brandlju/carla0.9.10.1/CarlaUE4/Content/Carla/Maps/OpenDrive/Town01.xodr"
    out_path = "./osm.osm"
    
    with open(map_path, "rb") as odrfile:
        rawmap = odrfile.read()
        # parse the byte encoded xml opendrive tree
        opendrive_byte = BytesIO(rawmap)
        opendrive = parse_opendrive(etree.parse(opendrive_byte).getroot())

        # convert opendrive to commonroad scenario
        scenario = convert_opendrive(opendrive)

        #print(scenario)

        # compute openstreetmap string representation of the commonroad scenario with right projection
        l2osm = L2OSMConverter(
           "+proj=omerc +lat_0=49 +lonc=8 +alpha=0 +k=1 +x_0=0 +y_0=0 +gamma=0 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0")
        openstreetmap = etree.tostring(l2osm(scenario), xml_declaration=True, encoding="UTF-8", pretty_print=True)

        with open(out_path ,"wb+") as mapfile:
            mapfile.write(openstreetmap)


    
    projection = lanelet2.projection.UtmProjector(lanelet2.io.Origin(49, 8))
    map = lanelet2.io.load(out_path, projection)

    make_positive(map.pointLayer)
    make_positive(map.lineStringLayer)
    make_positive(map.polygonLayer)
    make_positive(map.laneletLayer)
    make_positive(map.areaLayer)
    make_positive(map.regulatoryElementLayer)

    
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--participant",
        help="traffic participant type (one of vehicle, bicycle, pedestrian, train",
        type=str,
        required=False,
        default="vehicle")
    parser.add_argument("--lat", help="Lateral position of origin", type=float, default=49)
    parser.add_argument("--lon", help="Longitudinal position of origin", type=float, default=8)
    args = parser.parse_args()

    rules_map = {"vehicle": lanelet2.traffic_rules.Participants.Vehicle,
                 "bicycle": lanelet2.traffic_rules.Participants.Bicycle,
                 "pedestrian": lanelet2.traffic_rules.Participants.Pedestrian,
                 "train": lanelet2.traffic_rules.Participants.Train}
    proj = lanelet2.projection.UtmProjector(lanelet2.io.Origin(args.lat, args.lon))
    laneletmap = map

    routing_cost = lanelet2.routing.RoutingCostDistance(0.)  # zero cost for lane changes
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  rules_map[args.participant])
    graph = lanelet2.routing.RoutingGraph(laneletmap, traffic_rules, [routing_cost])
    debug_map = graph.getDebugLaneletMap()

    lanelet2.io.write("./test.osm", debug_map, proj)


    
if __name__ == "__main__":
    main()


