import copy

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import rospy
import json

try:
    mpl.use('Qt5Agg')
except ImportError:
    mpl.use('TkAgg')

from commonroad.common.file_reader import CommonRoadFileReader
from SMP.motion_planner.plot_config import StudentScriptPlotConfig

from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from derived_object_msgs.msg import ObjectArray
from custom_carla_msgs.msg import GlobalPathLanelets
from custom_carla_msgs.srv import UpdateLocalPath

from helper_functions import calc_egocar_yaw, calc_path_yaw

class LocalPlanner():

    def __init__(self, role_name):
        self.role_name = role_name
        self.current_pos = np.zeros(shape=2)
        self.current_orientation = None
        self.current_speed = None
        self.scenario = None
        self.planning_problem = None
        # self.global_path = None
        self.lanelets_on_route = None
        self.adjacent_lanelets = None
        self.adjacent_lanelets_flattened = None

        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_received)
        # self.global_path_sub = rospy.Subscriber(f"/psaf/{self.role_name}/global_path", Path, self.global_path_received)
        self.lanelet_sub = rospy.Subscriber(f"/psaf/{self.role_name}/global_path_lanelets", GlobalPathLanelets, self.lanelets_received)

        self.local_path_pub = rospy.Publisher(f"/psaf/{self.role_name}/local_path", Path, queue_size=1, latch=True)

    def map_received(self, msg):
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open()
        self.scenario.scenario_id = "DEU"
        self.org_scenario = copy.deepcopy(self.scenario)

    # def global_path_received(self, msg):
    #     self.global_path = msg

    def lanelets_received(self, msg):
        self.lanelets_on_route = msg.lanelet_ids
        self.adjacent_lanelets = json.loads(msg.adjacent_lanelet_ids)
        self.adjacent_lanelets_flattened = [item for sublist in self.adjacent_lanelets for item in sublist]

    def odometry_received(self, msg):
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.current_orientation = calc_egocar_yaw(msg.pose.pose)

    def _change_lane(self, current_lanelet, idx_nearest_point, left=False, right=False):
        if (left and right) or (not left and not right):
            raise ValueError("One of left and right must be true")
        path = []
        path.extend(current_lanelet.center_vertices[idx_nearest_point:idx_nearest_point + 10])
        if left:
            print("Lane change left")
            adj_lane = self.scenario.lanelet_network.find_lanelet_by_id(current_lanelet.adj_left)
        if right:
            print("Lane change right")
            adj_lane = self.scenario.lanelet_network.find_lanelet_by_id(current_lanelet.adj_right)
        path.extend(adj_lane.center_vertices[idx_nearest_point + 30:])
        return path

    def update_local_path(self, req):
        change_lane_left = req.change_lane_left
        change_lane_right = req.change_lane_right
        approach_intersection = req.approach_intersection
        leave_intersection = req.leave_intersection
        path = []
        possible_lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position([np.array(list(self.current_pos))])[0]
        for lane_id in possible_lanelet_ids:
            if lane_id in self.adjacent_lanelets_flattened:
                current_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lane_id)
                distances_to_center_vertices = np.linalg.norm(current_lanelet.center_vertices - self.current_pos,
                                                              axis=1)
                idx_nearest_point = np.argmin(distances_to_center_vertices)  # get nearest point in current lanelet
                if change_lane_left:
                    path.extend(self._change_lane(current_lanelet, idx_nearest_point, left=True))
                elif change_lane_right:
                    path.extend(self._change_lane(current_lanelet, idx_nearest_point, right=True))
                elif approach_intersection:
                    print("Approach Intersection")
                    for idx, sector in enumerate(self.adjacent_lanelets):
                        if lane_id in sector:
                            break
                    next_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(
                        self.adjacent_lanelets[idx + 1][0])
                    if next_lanelet.predecessor[0] == lane_id:  # no lane change
                        print("Keep Lane")
                        path.extend(current_lanelet.center_vertices[idx_nearest_point:])
                    elif next_lanelet.predecessor[0] == current_lanelet.adj_left:
                        path.extend(self._change_lane(current_lanelet, idx_nearest_point, left=True))
                    elif next_lanelet.predecessor[0] == current_lanelet.adj_right:
                        path.extend(self._change_lane(current_lanelet, idx_nearest_point, right=True))
                    print("Turn")
                    path.extend(next_lanelet.center_vertices[:])
                elif leave_intersection:
                    next_lanelet_id = current_lanelet.successor[0]
                    next_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(next_lanelet_id)
                    path.extend(current_lanelet.center_vertices[idx_nearest_point:])
                    path.extend(next_lanelet.center_vertices[:])
                else:
                    path.extend(current_lanelet.center_vertices[idx_nearest_point:])

        path = np.array(path)
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)

        self.local_path_pub.publish(path_msg)
        return True

if __name__ == "__main__":
    rospy.init_node('local_planner', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    lp = LocalPlanner(role_name)
    s = rospy.Service("update_local_path", UpdateLocalPath, lp.update_local_path)
    rospy.spin()
