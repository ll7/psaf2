from commonroad.scenario.lanelet import Lanelet
import rospy
import json
import numpy as np

from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from custom_carla_msgs.msg import GlobalPathLanelets, LaneStatus, NextLanelet

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

        self.lanelet_ids_roundabout_inside = None
        self.lanelet_ids_roundabout_incoming = None
        self.lanelet_ids_roundabout_outgoing = None
        self.lanelet_ids_roundabout_inside_outer_circle = None

        self.distance_pub = rospy.Publisher(f"/psaf/{self.role_name}/next_lanelet", NextLanelet, queue_size=1)
        # self.distance_pub = rospy.Publisher(f"/psaf/{self.role_name}/distance_next_intersection", Float64, queue_size=1)
        self.roundabout_pub = rospy.Publisher(f"/psaf/{self.role_name}/distance_next_roundabout", NextLanelet, queue_size=1)
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
        possible_ids = self.scenario.lanelet_network.find_lanelet_by_position([self.current_pos])
        if len(possible_ids) == 0:  # no id found
            rospy.loginfo("Car is not on street. Abort.")
            ls.currentLaneId = -1
            self.lane_status_pub.publish(ls)
            return

        current_lanelet_id = -1
        for lanelet_id in possible_ids[0]:
            if lanelet_id in self.adjacent_lanelets_flattened:  # check if lanelet is on global path
                current_lanelet_id = lanelet_id
                ls.currentLaneId = current_lanelet_id
                break

        next_lanelet_msg = NextLanelet()
        lane = self.scenario.lanelet_network.find_lanelet_by_id(current_lanelet_id)
        min_distance_to_outer = np.inf

        if lane is None:
            next_lanelet_msg.distance = np.inf
            next_lanelet_msg.isInIntersection = False
            distance = 0
        else:
            distances_to_center_vertices = np.linalg.norm(lane.center_vertices - self.current_pos, axis=1)
            idx = np.argmin(distances_to_center_vertices)
            next_lanelet_msg.distance = self.lanelet_lengths[current_lanelet_id][-1] - \
                                        self.lanelet_lengths[current_lanelet_id][idx]
            if current_lanelet_id in self.intersection_lanelet_ids or current_lanelet_id in self.lanelet_ids_roundabout_inside:
                distance = np.inf
                next_lanelet_msg.isInIntersection = False
            elif self.lanelet_ids_roundabout_incoming is not None:
                # next_lanelet_msg.isRoundabout = True
                distance = np.inf
                if current_lanelet_id in self.lanelet_ids_roundabout_incoming:
                    # rospy.loginfo("current_lanelet_id is in incoming_roundabout_lanes")
                    point_with_min_distance = None
                    for inner_lanelet_id in self.lanelet_ids_roundabout_inside_outer_circle:
                        inner_lane = self.scenario.lanelet_network.find_lanelet_by_id(inner_lanelet_id)
                        distances_to_right_vertices = np.linalg.norm(inner_lane.right_vertices - self.current_pos, axis=1)
                        _min = np.min(distances_to_right_vertices)
                        if _min < min_distance_to_outer:
                            min_distance_to_outer = _min
                            point_with_min_distance = inner_lane.right_vertices[np.argmin(distances_to_right_vertices)]
            else:
                next_lanelet_msg.isInIntersection = True
                distance = self.lanelet_lengths[current_lanelet_id][-1] - self.lanelet_lengths[current_lanelet_id][idx]
                if lane.adj_left_same_direction:
                    ls.isMultiLane = True
                    ls.leftLaneId = lane.adj_left
                if lane.adj_right_same_direction:
                    ls.isMultiLane = True
                    ls.rightLaneId = lane.adj_right
        if min_distance_to_outer < 20:
            # rospy.loginfo("On Lanelet to RoundAbout")
            roundabout_msg = NextLanelet()
            roundabout_msg.isRoundabout = True
            roundabout_msg.entry_point = Point(point_with_min_distance[0], point_with_min_distance[1], 0)
            # rospy.loginfo("pub roundaboutmsg ")
            self.roundabout_pub.publish(roundabout_msg)
            # self.distance_pub.publish(np.inf)
        else:
            self.distance_pub.publish(next_lanelet_msg)
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
    rospy.init_node('road_features', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    tf = TrafficFeatures(role_name)
    tf.run()


if __name__ == "__main__":
    main()
