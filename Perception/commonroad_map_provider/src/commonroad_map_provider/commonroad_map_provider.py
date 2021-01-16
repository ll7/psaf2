import io
import rospy

from lxml import etree

from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.network import Network

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag

from carla_msgs.msg import CarlaWorldInfo
from std_msgs.msg import String


class CommonroadMapProvider:
    """
    Class to subscribe /carla/WorldInfo topic and create /psaf/LaneletMap topic.
    """
    
    def __init__(self, role_name):
        self.role_name = role_name
        self.sub = rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.convert_open_drive_map)
        self.commonroad_publisher = rospy.Publisher(f"/psaf/{self.role_name}/commonroad_map", String, queue_size=1, latch=True)
 
    def convert_open_drive_map(self, msg):  
        # Import, parse and convert OpenDRIVE file
        opendrive_byte = io.BytesIO(msg.opendrive.encode())
        open_drive = parse_opendrive(etree.parse(opendrive_byte).getroot())

        road_network = Network()
        road_network.load_opendrive(open_drive)

        scenario = road_network.export_commonroad_scenario()
        # Write CommonRoad scenario to file
        writer = CommonRoadFileWriter(
            scenario=scenario,
            planning_problem_set=PlanningProblemSet(),
            author="",
            affiliation="",
            source="OpenDRIVE 2 Lanelet Converter",
            tags={Tag.URBAN, Tag.HIGHWAY},
        )
        writer.write_to_file("./commonroad_map.xml", OverwriteExistingFile.ALWAYS)
        print("DONE")
        self.commonroad_publisher.publish("./commonroad_map.xml")

    
if __name__ == "__main__":
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    com = CommonroadMapProvider(role_name)
    
    # idle so topic is still present
    # (resp. if world changes)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass









