import carla
import rospy
import numpy as np
from std_msgs.msg import Float64, Float32, Int8
from custom_carla_msgs.msg import PerceptionInfo


class BehaviorTrafficLights():

    def __init__(self, role_name):
        self.role_name = role_name
        self.current_speed = 0
        self.state = True
        self.stop_dist_intersec = np.inf
        self.toplinie_dist = np.inf
        self.braking_distance = np.inf

        # SUBSCRIBERS
        self.intersection_sub = rospy.Subscriber(f"/psaf/{self.role_name}/distance_next_intersection", Float64,
                                                 self.next_intersection_info)
        self.stopline_sub = rospy.Subscriber(f"/psaf/{self.role_name}/stopline_distance", Float64, 
                                                self.stopline_distance_info)
        self.traffic_light_sub = rospy.Subscriber(f"/psaf/{self.role_name}/perception_info", PerceptionInfo,
                                                  self.traffic_light_info)
        #self._target_speed_subscriber = rospy.Subscriber("/carla/{}/target_speed".format(role_name), Float64,
        #                                                 self.update_target_speed)
        #self.speed_sub = rospy.Subscriber(f"/psaf/{self.role_name}/vehicle_status", Float32,
        #                                  self.speed_update)
        #self.radar_sub = rospy.Subscriber(
        #    f"/carla/{role_name}/radar/front/radar_points", PointCloud2, self.radar_updated)

        # PUBLISHERS
        self.entry_state_pub = rospy.Publisher("/carla/{}/vehicle_entry_state_interseption".format(role_name), Int8, queue_size=1)
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
        #self.traffic_stop_pub = rospy.Publisher("/psaf/{}/traffic_stop_dist".format(role_name), Float64, queue_size=1)

    def traffic_light_info(self, perception):
        """
        callback on traffic light state
        """
        red = 0
        yellow = 0
        green = 0
        for x in perception.values: 
            if x == "red":
                red = red + 1
            elif x == "yellow":
                yellow = yellow + 1
            elif x == "green":
                green = green + 1
        if red > green and red > yellow:
            self.stop()
        elif yellow > red and yellow > green:
            self.check()
        else:
            self.go()

    def next_intersection_info(self, intersection_dist):
        """
        callback on distance_next_intersection
        """
        self.stop_dist_intersec = intersection_dist.data

    
    def stopline_distance_info(self, stopline_dist):
        """
        callback on distance_next_intersection
        """
        self.stoplinie_dist = stopline_dist.data

    #def vehicle_front_info(self, msg):
    #    """
    #    callback on vehicle front
    #    """
    #    stop_dist_vehicle = 20 #test value
    #    return stop_dist_vehicle

    #def speed_update (self, speed):
    #    self.actual_speed = speed.velocity
    #    if self.state == False:
    #        target_speed = 0
    #        self.target_speed_pub.publish(target_speed)


    def stop(self):
        """
        callback vehicle_stop
        """
        rospy.loginfo("Vehicle STOP")
        #self.state = False
        target_speed = 0
        self.target_speed_pub.publish(target_speed)


    def go(self):
        """
        callback vehicle_run
        """
        rospy.loginfo("Vehicle RUN")
        #self.state = True
        target_speed = 5
        self.target_speed_pub.publish(target_speed)

    
    def check(self):
        rospy.loginfo("CHECK")
        stoplinie_dist = self.stoplinie_dist
        stop_dist_intersec = self.stop_dist_intersec
        stop_distance = min(stoplinie_dist, stop_dist_intersec) # maximum distance to stop
        #rospy.loginfo(str(stop_distance), str(self.stoplinie_dist))
        braking_distance = 70 #pow((self.current_speed/10), 2)
        print("braking_distance:", braking_distance, "stoplinie_dist:", stoplinie_dist, "intersection_distance:", stop_dist_intersec)
        if (braking_distance + 1.5) < stop_distance:
            self.go()
        else:
            self.stop()

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