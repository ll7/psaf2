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
                                                         self.update_target_speed)
        #self._odometry_subscriber = rospy.Subscriber("/carla/{}/speedometer".format(role_name), Float32, self.speed)

        # PUBLISHERS
        # self.entry_state_pub = rospy.Publisher("/carla/{}/vehicle_entry_state_interseption".format(role_name), Int8, queue_size=1)
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)

    def update_target_speed(self, speed):
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
        #"""
        #callback on distance_stop_line
        #"""
        print('if there is a stop line call an external function')

    def traffic_light_info (self, perception):
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

    def go(self):
        rospy.loginfo("RUN")

    def vehicle_front (self, msg):
        """
         callback on vehicle front
        """
        #self.dictance_to_vehicle = msg.data

        # NUR FÜR TEST:
        client = carla.Client(args.host, args.port)
        world = client.load_world('Town01')
        blueprints = world.get_blueprint_library().filter(args.filterv)
        #get current vehicle transform to get both location & rotation
        #tf = current_v.get_tranform()
        #Spawn new vehicle with some distance addition to above transformation location (say 10meter in x, y)
        new_loc = carla.Location(99, 136.4, 0)
        #world.spawn(new_vehicle, carla.Transform(new_loc, carla.Rotation()))
        #transform = Transform(Location(x=99, y=136.4, z=0), Rotation(yaw=180))
        actor = world.spawn_actor(blueprint, new_loc)


        #return self.dictance_to_vehicle
    
    def update_speed (self, speed):
        print(' ')

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



