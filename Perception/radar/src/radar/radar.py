import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pc2


class Radar(object):  # pylint: disable=too-few-public-methods
    """
    VehicleController is the combination of two controllers (lateral and longitudinal)
    to perform the low level control a vehicle from client side
    """

    def __init__(self, role_name, target_speed, args_longitudinal=None, args_lateral=None):

        self.role_name = role_name
        self.points = []
        self._route_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/radar/front/radar_points", PointCloud2, self.radar_updated)
        self._dist_publisher = rospy.Publisher(f"psaf/{role_name}/radar/distance", Float64, queue_size=1)
    
    
    def radar_updated(self, msg):
        self.points = pc2.read_points(msg, skip_nans=True, field_names=("x","y","z"))
        dist = [p[0] for p in self.points]
        if len(dist) > 0:
            self._dist_publisher.publish(min(dist))
        


    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass

def main():
    rospy.init_node('radar', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    target_speed = rospy.get_param("~target_speed", 0)
    radar = Radar(role_name, target_speed)
    try:
        radar.run()
    finally:
        del radar
    rospy.loginfo("Done")

if __name__ == "__main__":
    main()



