from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from time import sleep
import rospy
from helper_functions import distance_between_points


class ScriptedControl:

    def __init__(self, role_name):
        self.role_name = role_name
        self.vehicle_control_manual_override_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            Bool, queue_size=1, latch=True)
        self.vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd_manual".format(self.role_name),
            CarlaEgoVehicleControl, queue_size=1)

        self._control = CarlaEgoVehicleControl()

    def run(self):
        #step one: start driving
        # self._control.throttle = 1.
        # self._control.hand_brake = 0
        # try:
        #     self.vehicle_control_publisher.publish(self._control)
        # except rospy.ROSException as error:
        #     rospy.logwarn("Could not send vehicle control: {}".format(error))
        #
        # sleep(3)
        #
        # #step two: go right
        # self._control.steer = -1.
        # try:
        #     self.vehicle_control_publisher.publish(self._control)
        # except rospy.ROSException as error:
        #     rospy.logwarn("Could not send vehicle control: {}".format(error))
        #
        # sleep(3)
        point1 = Point()
        point1.x = 1
        point1.y = 1
        point1.z = 1
        point2 = Point()
        point2.x = 2
        point2.y = 2
        point2.z = 2
        print("distance between point1 & point2  = {} in ackermann".format(distance_between_points(point1, point2)))        
        self.vehicle_control_manual_override_publisher.publish((Bool(data=True)))
        while True:
            sleep(0.2)
            self._control.throttle = 1.
            try:
                self.vehicle_control_publisher.publish(self._control)
            except rospy.ROSException as error:
                rospy.logwarn("Could not send vehicle control: {}".format(error))


def main():
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    c = ScriptedControl(role_name)
    c.run()
    

if __name__ == '__main__':    
    main()

