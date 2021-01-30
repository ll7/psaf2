from commonroad.scenario.lanelet import Lanelet
import rospy

import numpy as np

from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry

from commonroad.common.file_reader import CommonRoadFileReader

class TrafficFeatures:
    def __init__(self, role_name):
        self.role_name = role_name
        self.scenario = None
        self.planning_problem_set = None
        self.current_pos = np.zeros(2)

        
        self.distance_pub = rospy.Publisher(f"/psaf/{self.role_name}/distance", Float64, queue_size=1)
        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_received)


    def odometry_received(self, msg):
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])


    def map_received(self, msg):
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open()
        self.scenario.scenario_id = "DEU"

    
    def update_distance(self):
        id = self.scenario.lanelet_network.find_lanelet_by_position([self.current_pos])
        if len(id) == 0:
            print("Wir haben ein Problem")
            distance = 0
        else:
            lane = self.scenario.lanelet_network.find_lanelet_by_id(id[0][0])
            last_point = lane._center_vertices[-1]
            distance = np.sqrt((self.current_pos[0] - last_point[0])**2 + (self.current_pos[1] - last_point[1])**2)
        self.distance_pub.publish(distance)      


    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.scenario:
                self.update_distance()
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass



def main():
    rospy.init_node('traffic_features', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    tf = TrafficFeatures(role_name)
    tf.run()


if __name__ == "__main__":
    main()
