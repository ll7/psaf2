import os
import sys
import io
import rospy
import time
from carla_msgs.msg import CarlaWorldInfo
from custom_carla_msgs.msg import LaneletMap



class StanleyControl:
    """
    Check if the right thing was published by the lanelet_map_provider.
    Wirtes two files in .ros: "published_test_map.osm" and "org_test_map.osm".
    If files are the same excpet for the ids the everything worked fine.
    You can check the results graphically in josm-editor.
    """
    def __init__(self, role_name):
        self.role_name = role_name
      # self.lanelet_sub = rospy.Subscriber("/psaf/lanelet_map", LaneletMap, self.map_recieved)
      #  self.osm_sub = rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.load_opendrive)

       # self.org_lanelet_map = None
       # self.published_lanelet_map = None


    
if __name__ == "__main__":
    rospy.init_node('stanley_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    
    # idle so topic is still present
    # (resp. if world changes)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

