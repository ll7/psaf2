import rospy
from sensor_msgs.msg import NavSatFix

class TargetPublisher:
    def __init__(self, role_name):
        
        self.target_pub = rospy.Publisher(f"/psaf/{role_name}target_point", NavSatFix, queue_size=1, latch=True)
        target = NavSatFix()
        target.latitude = -0.0016077391132190087
        target.longitude = -8.324308136330368e-05
        target.altitude = 1.965698003768921
        self.target_pub.publish(target)


if __name__ == "__main__":
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    gp = TargetPublisher(role_name)

    # idle so topic is still present
    # (resp. if world changes)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
