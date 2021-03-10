import carla
import rospy
from std_msgs.msg import Float64, Float32, Int8
from custom_carla_msgs.msg import PerceptionInfo


class BehaviorTrafficLights():

    def __init__(self, role_name):
        self.role_name = role_name
        self.actual_speed = 0

        # SUBSCRIBERS
        self.intersection_sub = rospy.Subscriber(f"/psaf/{self.role_name}/distance_next_intersection", Float64,
                                                 self.next_intersection_info)
        self.traffic_light_sub = rospy.Subscriber(f"/psaf/{self.role_name}/perception_info", PerceptionInfo,
                                                  self.traffic_light_info)
        self.intersection_dist__sub = rospy.Subscriber(f"/psaf/{self.role_name}/distance_next_intersection", Float64,
                                                       self.next_intersection_info)
        #self._target_speed_subscriber = rospy.Subscriber("/carla/{}/target_speed".format(role_name), Float64,
        #                                                 self.update_target_speed)
        self.speed_sub = rospy.Subscriber(f"/psaf/{self.role_name}/vehicle_status", Float32,
                                          self.speed_update)
        #self.radar_sub = rospy.Subscriber(
        #    f"/carla/{role_name}/radar/front/radar_points", PointCloud2, self.radar_updated)

        # PUBLISHERS
        self.entry_state_pub = rospy.Publisher("/carla/{}/vehicle_entry_state_interseption".format(role_name), Int8, queue_size=1)
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
        self.traffic_stop_pub = rospy.Publisher("/psaf/{}/traffic_stop_dist".format(role_name), Float64, queue_size=1)

    def traffic_light_info(self, perception):
        """
        callback on traffic light state
        """
        x_coord = 1
        for x in perception.relative_x_coord:
            coord = abs(0.5 - x)
            if coord < x_coord:
                x_coord = coord
                light_nr = perception.relative_x_coord.index(x)
        #if ('red' in perception.values) or ('yellow' in perception.values):
        if len(perception.values) != 0:
            if (perception.values[light_nr] == 'red') or (perception.values[light_nr] == "yellow"):
                self.stop()
        else:
            self.go()

    def next_intersection_info(self, intersection_dist):
        """
        callback on distance_next_intersection
        """
        stop_dist_intersec = intersection_dist.data

        return stop_dist_intersec

    def vehicle_front_info(self, msg):
        """
        callback on vehicle front
        """
        stop_dist_vehicle = 20 #test value
        return stop_dist_vehicle

    def speed_update (self, speed):
        self.actual_speed = speed.velocity

    def stop(self):
        rospy.loginfo("STOP")
        self.stop_dist_intersec = 8 # for test
        self.stop_dist_vehicle = 7 # for test
        self.stop_distance = min(self.stop_dist_intersec, self.stop_dist_vehicle) # maximum distance to stop
        self.current_speed = self.actual_speed
        braking_distances = pow((self.current_speed/10), 2)
        if braking_distances <= self.stop_distance:
            target_speed = 0
            self.target_speed_pub.publish(target_speed)

    def go(self):
        """
        callback on target speed
        """
        rospy.loginfo("RUN")
        target_speed = 5
        self.target_speed_pub.publish(target_speed)

    def run(self):
        """
        drive into the intersection
        """
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
