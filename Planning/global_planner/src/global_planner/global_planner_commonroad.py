import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import json

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from helper_functions import calc_egocar_yaw
from SMP.route_planner.route_planner.route_planner import RoutePlanner
from SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path, \
    get_plot_limits_from_routes
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.trajectory import State
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle
from commonroad.visualization.plot_helper import *

from carla_msgs.msg import CarlaWorldInfo
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from custom_carla_msgs.msg import GlobalPathLanelets
from tf.transformations import euler_from_quaternion

PLOT_CROSSING = False


class GlobalPlanner:
    # TODO: GOAL AS GPS POINT
    def __init__(self, role_name):
        self.role_name = role_name
        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.target_sub = rospy.Subscriber(f"/psaf/{self.role_name}/target_point", NavSatFix, self.target_received)
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_received)
        self.inital_pose_sub = rospy.Subscriber(f"/initialpose", PoseWithCovarianceStamped, self.init_pose_received)
        self.sub = rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.world_info_received)

        self.path_pub = rospy.Publisher(f"/psaf/{self.role_name}/global_path", Path, queue_size=1, latch=True)
        self.lanelet_pub = rospy.Publisher(f"/psaf/{self.role_name}/global_path_lanelets", GlobalPathLanelets,
                                           queue_size=1, latch=True)

        self.scenario = None
        self.planning_problem_Set = None
        self.current_pos = None
        self.current_orientation = None
        self.intersection_lanelet_ids = None

    def map_received(self, msg):
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open()
        self.scenario.scenario_id = "DEU"
        self.publish_intersection_lanelet_ids(msg.data)
        while self.scenario is None or self.current_orientation is None:
            rospy.sleep(0.05)
        self.create_global_plan()

    def target_received(self, msg):
        print("Target Received")
        self.target_gnss = msg

    def init_pose_received(self, pose):
        print("New Start")
        while self.scenario is None or self.current_orientation is None:
            rospy.sleep(0.05)
        rospy.sleep(0.2)
        self.create_global_plan()

    def odometry_received(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_orientation = msg.pose.pose

    def world_info_received(self, msg):
        self.map_number = msg.map_name

    def publish_intersection_lanelet_ids(self, map_name):
        def plot_map(intersections, plot_labels=True):
            plt.figure(figsize=(20, 10))
            # plot the scenario at different time step
            handles = draw_object(self.scenario)
            plt.gca().set_aspect('equal')

            for id_lanelet in intersections:
                lanelet = self.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
                draw_object(lanelet, handles=handles, draw_params={'lanelet': {
                    'unique_colors': False,  # colorizes center_vertices and labels of each lanelet differently
                    'draw_stop_line': False,
                    'stop_line_color': '#ffffff',
                    'draw_line_markings': True,
                    'draw_left_bound': False,
                    'draw_right_bound': False,
                    'draw_center_bound': True,
                    'draw_border_vertices': False,
                    'draw_start_and_direction': True,
                    'show_label': plot_labels,
                    'draw_linewidth': 1,
                    'fill_lanelet': True,
                    'facecolor': '#00b8cc',  # color for filling
                    'zorder': 30,  # put it higher in the plot, to make it visible
                    'center_bound_color': '#3232ff',  # color of the found route with arrow
                }})

            plt.show()

        # every lanelet in a intersection has exactly one successor and predecessor
        intersection_ids = []
        for lanelet_id in range(100, len(self.scenario.lanelet_network.lanelets) + 100):
            lane = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            if len(lane.predecessor) == 1 and len(lane.successor) == 1 and lane.distance[-1] < 100:
                intersection_ids.append(lanelet_id)

        # map nummer 3 ist die mit kreisverkehr
        if self.map_number == 1:
            ids_to_remove = []
        elif self.map_number == 2:
            ids_to_remove = []
        elif self.map_number == 3:
            ids_to_remove = [115, 107, 342, 340, 176, 169, 258, 259, 260, 320, 386, 177]            
        elif self.map_number == 5:
            ids_to_remove = [256, 252, 255, 258, 354, 259, 383, 377, 374, 384, 381, 376, 254]                            
        else:
            ids_to_remove = []

        self.lanelet_ids_roundabout_inside = [190, 191, 196, 306, 308, 305, 307, 198, 201, 202, 199, 195, 188, 193]
        self.lanelet_ids_roundabout_incoming = [184, 181, 203, 280, 284, 194, 189, 347, 186, 183, 281, 277]
        self.lanelet_ids_roundabout_outgoing = [205, 200, 204, 188, 199, 345, 206, 187]
        self.anelet_ids_roundabout_inside_inner_circle = [190, 191, 306, 305, 198, 199, 188]
        self.lanelet_ids_roundabout_inside_outer_circle = [202, 201, 195, 307, 308, 196, 193]

        out_list = []
        for id in intersection_ids:
            if id not in ids_to_remove:        
                out_list.append(id)
        # and id not in lanelet_ids_roundabout_incoming
        # and id not in self.lanelet_ids_roundabout_inside and id not in self.lanelet_ids_roundabout_outgoing
        if PLOT_CROSSING:
            plot_map(out_list, False)

        self.intersection_lanelet_ids = out_list

    def create_global_plan(self):
        print("Path creation started")
        goal_state = State(position=Rectangle(2, 2, center=np.array([10, 50])), time_step=Interval(1, 200), velocity=Interval(0, 0), orientation=AngleInterval(-0.2, 0.2))
        goal_region = GoalRegion([goal_state])

        yaw = calc_egocar_yaw(self.current_orientation)
        start_state = State(position=np.array([self.current_pos[0], self.current_pos[1]]), time_step=1, velocity=0.0, yaw_rate=0.0, slip_angle=0.0, orientation=yaw)

        planning_problem = PlanningProblem(10000, start_state, goal_region)
        # self.planning_problem_set.add_planning_problem(planning_problem)

        # instantiate a route planner

        route_planner = RoutePlanner(self.scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)

        # plan routes, and save the found routes in a route candidate holder
        candidate_holder = route_planner.plan_routes()

        # we retrieve the first route in the list
        # this is equivalent to: route = list_routes[0]
        route = candidate_holder.retrieve_first_route()
        # print(f"Time: {time.time() -start}")

        lanelet_msg = GlobalPathLanelets()
        lanelet_msg.lanelet_ids = route.list_ids_lanelets
        lanelet_msg.adjacent_lanelet_ids = json.dumps(route.retrieve_route_sections())
        lanelet_msg.lanelet_ids_in_intersection = self.intersection_lanelet_ids

        lanelet_msg.lanelet_ids_roundabout_inside = self.lanelet_ids_roundabout_inside
        lanelet_msg.lanelet_ids_roundabout_incoming = self.lanelet_ids_roundabout_incoming
        lanelet_msg.lanelet_ids_roundabout_outgoing = self.lanelet_ids_roundabout_outgoing
        lanelet_msg.lanelet_ids_roundabout_inside_outer_circle = self.lanelet_ids_roundabout_inside_outer_circle       

        self.lanelet_pub.publish(lanelet_msg)

        point_route = route.reference_path
        point_route = self.sanitize_route(point_route)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for p in point_route:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # #plotting
        # plt.cla()
        # plt.clf()
        # plt.ion()
        # # determine the figure size for better visualization
        # plot_limits = get_plot_limits_from_routes(route)
        # size_x = 6
        # ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
        # plt.gca().axis('equal')
        # draw_route(route, draw_route_lanelets=True, draw_reference_path=True, plot_limits=plot_limits)
        # plt.pause(0.001)


    def sanitize_route(self, route):
        # delete unnecessary points at begin and end of route
        max_index = -1
        min_index = -1
        min_distance_start = None
        min_distance_end = None


        # find last point before start position
        for i, point in enumerate(route):
            distance, _ = self.compute_magnitude_angle(point, self.current_pos, self.current_orientation.orientation)

            if min_distance_start is not None:
                if distance < min_distance_start:
                    max_index = i
                    min_distance_start = distance
            else:
                min_distance_start = distance

        # delete points before start position
        if max_index >= 0:
            cut_left = max_index + 1
        else:
            cut_left = 0

        # find last point after target position
        #TODO: MAKE DYNAMIC
        for i, point in enumerate(reversed(route)):
            distance, _ = self.compute_magnitude_angle(point, np.array([10, 50]), self.current_orientation.orientation)

            if min_distance_end is not None:
                if distance < min_distance_end:
                    min_index = i
                    min_distance_end = distance
            else:
                min_distance_end = distance

        # delete points after target position
        if min_index >= 0:
            cut_right = (min_index + 1) * -1
        else:
            cut_right = -1

        route = route[cut_left:cut_right]
        return route

    def compute_magnitude_angle(self, target_location, current_location, qorientation):
        """
        Compute relative angle and distance between a target_location and a current_location
        """
        # angle of vehicle
        quaternion = (
            qorientation.x,
            qorientation.y,
            qorientation.z,
            qorientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        orientation = math.degrees(-yaw)

        # vector from vehicle to target point and distance
        target_vector = np.array([target_location[0] - current_location[0], target_location[1] - current_location[1]])
        norm_target = np.linalg.norm(target_vector)

        # vector of the car and absolut angle between vehicle and target point
        forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
        d_angle = math.degrees(math.acos(np.dot(forward_vector, target_vector) / norm_target))

        # make angle negative or positive
        cross = np.cross(forward_vector, target_vector)
        if cross > 0:
            d_angle *= -1.0

        return (norm_target, d_angle)



if __name__ == "__main__":
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    gp = GlobalPlanner(role_name)

    # idle so topic is still present
    # (resp. if world changes)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
