import carla
import rospy
from std_msgs.msg import Float64, Float32, Int8
from custom_carla_msgs.msg import PerceptionInfo


class BehaviorTrafficLights():

    def __init__(self, role_name):
        self.role_name = role_name

        # SUBSCRIBERS
        self.intersection_sub = rospy.Subscriber(f"/psaf/{self.role_name}/distance_next_intersection", Float64,
                                                 self.next_intersection_info)
        self.traffic_light_sub = rospy.Subscriber(f"/psaf/{self.role_name}/perception_info", PerceptionInfo,
                                                  self.traffic_light_info)
        self.radarsubscriber = rospy.Subscriber(f"psaf/{role_name}/radar/distance", Float64,
                                                self.radar_info)

        self._target_speed_subscriber = rospy.Subscriber("/carla/{}/target_speed".format(role_name), Float64,
                                                         self.target_speed)
        #self._odometry_subscriber = rospy.Subscriber("/carla/{}/speedometer".format(role_name), Float32, self.speed)

        # PUBLISHERS
        # self.entry_state_pub = rospy.Publisher("/carla/{}/vehicle_entry_state_interseption".format(role_name), Int8, queue_size=1)
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)

    def target_speed(self, speed):
        """
        callback on target speed
        """
        target_speed = speed.data

    def next_intersection_info (self, distance_intersection):
        """
        callback on distance_next_intersection
        """
        self.distance_inter = distance_intersection.data
        return self.distance_inter

    def istance_stop_line(self, distance_stop):
        """
        callback on distance_stop_line
        """
        self.distance_stop_line = distance_stop.data
        return self.distance_stop_line

    def traffic_light_info (self, perception):
        """
        callback on traffic light state
        """
        speed = 5
        print(perception.values)
        if ('red' in perception.values) or ('yellow' in perception.values):
        #if (perception.values == ['red']) or (light == "yellow"):
            self.stop()
            rospy.loginfo("STOP")
        else:
            self.no_red()
            #runn(self)
        return speed

    def no_red(self):
        rospy.loginfo("RUN")

    def radar_info (self, msg):
        """
         callback on vehicle front
        """
        self.dictance_to_vehicle = msg.data
        return self.dictance_to_vehicle
    
    def update_speed (self, speed):
        print(speed)

    def stop (self):
        rospy.loginfo("STOP")

    def run(self):
        """
        drive into the intersection
        """
        #if self.traffic_light_info():
            #if self.dictance_to_vehicle > 0:
                #distance = self.dictance_to_vehicle
            #elif self.distance_stop_line > 0:
                #distance = self.distance_stop_line
            #else:
                #distance = self.distance_inter

            #self.stop(distance)
        #else:
            #self.run()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass


def main():
    rospy.init_node('traffic_lights_intersection', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    #target_speed = rospy.get_param("~target_speed", 0)
    tf = BehaviorTrafficLights(role_name)
    tf.run()


if __name__ == '__main__':
    main()



