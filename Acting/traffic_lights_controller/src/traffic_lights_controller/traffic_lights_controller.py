
from carla_msgs.msg import CarlaTrafficLightInfo, CarlaTrafficLightStatus, CarlaTrafficLightStatusList, CarlaEgoVehicleControl
#from carla_ros_bridge.actor import Actor
import carla
import rospy

class TrafficLightsControl():

    def __init__(self, world, role_name):
        self.vehicle = None
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')
        self.world = world

        self.vehicle_brake_publisher = rospy.Publisher(
            "/carla/{}/vehicle_brake_state".format(role_name), CarlaEgoVehicleControl, queue_size=1)
            #"/carla/{}/vehicle_control_cmd".format(role_name), CarlaEgoVehicleControl, queue_size=1)
        #self.activ_traffic_light_publisher = rospy.Publisher(
             #"/carla/{}/activ_traffic_light".format(role_name), CarlaEgoVehicleControl, queue_size=1)

    def run(self):
            """
            Look for an carla actor with name 'ego_vehicle' 
            Changes status on red traffic light
            """

            #t_status = CarlaTrafficLightStatus() #
            #t_status.id = 0
            #self.activ_traffic_light_publisher.publish(t_status)

            v_status = CarlaEgoVehicleControl()
            #rospy.loginfo("Waiting for ego vehicle...")
            for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == self.role_name:
                    self.ego_vehicle = actor
                    rospy.loginfo("Ego vehicle found.")
                    v_status.brake = 0.0
                    rospy.loginfo("Init brake_status: {}".format(v_status.brake))
                    break
            while True:
            #Get the traffic light affecting a vehicle
                while self.ego_vehicle.is_at_traffic_light() == True:
                    traffic_light = self.ego_vehicle.get_traffic_light()
                    if (traffic_light.get_state() == carla.TrafficLightState.Red): #or (traffic_light.get_state() == carla.TrafficLightState.Yellow):
                       v_status.brake = 1.0 
                       rospy.loginfo("NEW Brake_Status: {}".format(v_status.brake))
                       self.vehicle_brake_publisher.publish(v_status)
                    else:
                        v_status.brake = 0.0
                        self.vehicle_brake_publisher.publish(v_status)
                v_status.brake = 0.0  
                rospy.loginfo("Braske_Status: {}".format(v_status.brake))
                self.vehicle_brake_publisher.publish(v_status)

                
                



def main():
    rospy.init_node('brake_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    host = rospy.get_param("host", "127.0.0.1")
    port = rospy.get_param("port", 2000)

    carla_client = carla.Client(host=host, port=port)
    world = carla_client.get_world()
    lt = TrafficLightsControl(world,role_name)

    try:
        lt.run()
    finally:
        del lt
    rospy.loginfo("Done")

if __name__ == '__main__':
    main()
            

