from commonroad.scenario.lanelet import Lanelet
import rospy
import json
import numpy as np

from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from custom_carla_msgs.msg import GlobalPathLanelets, LaneStatus

from commonroad.common.file_reader import CommonRoadFileReader


class TrafficFeatures:
    def __init__(self, role_name):
        self.role_name = role_name
        self.scenario = None
        self.planning_problem_set = None
        self.current_pos = np.zeros(2)
        self.adjacent_lanelets_flattened = []
        self.lanelet_lengths = {}
        self.last_non_intersection_lanelet_id = None
        
        self.distance_pub = rospy.Publisher(f"/psaf/{self.role_name}/distance_next_intersection", Float64, queue_size=1)
        self.distance_roundabout_pub = rospy.Publisher(f"/psaf/{self.role_name}/distance_next_roundabout", Float64, queue_size=1)
        self.lane_status_pub = rospy.Publisher(f"/psaf/{self.role_name}/lane_status", LaneStatus, queue_size=1)
        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_received)
        self.lanelet_sub = rospy.Subscriber(f"/psaf/{self.role_name}/global_path_lanelets", GlobalPathLanelets, self.lanelets_received)

    def odometry_received(self, msg):
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def map_received(self, msg):
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open()
        self.scenario.scenario_id = "DEU"
        if self.scenario and self.adjacent_lanelets_flattened:
            self.update_lanelet_lengths()

    def lanelets_received(self, msg):
        self.lanelets_on_route = msg.lanelet_ids
        self.adjacent_lanelets = json.loads(msg.adjacent_lanelet_ids)
        self.adjacent_lanelets_flattened = [item for sublist in self.adjacent_lanelets for item in sublist]
        self.intersection_lanelet_ids = msg.lanelet_ids_in_intersection

        self.lanelet_ids_roundabout_inside = msg.lanelet_ids_roundabout_inside
        self.lanelet_ids_roundabout_incoming = msg.lanelet_ids_roundabout_incoming
        self.lanelet_ids_roundabout_outgoing = msg.lanelet_ids_roundabout_outgoing
        self.lanelet_ids_roundabout_inside_outer_circle = msg.lanelet_ids_roundabout_inside_outer_circle

        if self.scenario and self.adjacent_lanelets_flattened:
            self.update_lanelet_lengths()

    def update_lanelet_lengths(self):
        for lanelet_id in self.adjacent_lanelets_flattened:
            lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            self.lanelet_lengths[lanelet_id] = lanelet.distance

    def update_road_features(self):
        # Create LaneStatus message
        ls = LaneStatus()
        ls.isMultiLane = False
        ls.rightLaneId = -1
        ls.leftLaneId = -1

        # find lanelet id. Only lanelets that are on the global path are considered.
        possible_ids = self.scenario.lanelet_network.find_lanelet_by_position([self.current_p
                    distances_to_outer_circle.append(np.argmin(distances_to_center_vertices))
                if len(distances_to_outer_circle) > 0:
                    distance = min(distances_to_outer_circle)

            else:
                distances_to_center_vertices = np.linalg.norm(lane.center_vertices - self.current_pos, axis=1)
                idx = np.argmin(distances_to_center_vertices)
                distance = self.lanelet_lengths[current_lanelet_id][-1] - self.lanelet_lengths[current_lanelet_id][idx]
                if lane.adj_left_same_direction:
                    ls.isMultiLane = True
                    ls.leftLaneId = lane.adj_left
                if lane.adj_right_same_direction:
                    ls.isMultiLane = True
                    ls.rightLaneId = lane.adj_right

        
        if current_lanelet_id in self.lanelet_ids_roundabout_incoming:
            self.distance_roundabout_pub.publish(distance)
            self.distance_pub.publish(np.inf)
        else:
            self.distance_pub.publish(distance)
        self.lane_status_pub.publish(ls)

    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.scenario:
                self.update_road_features()
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
