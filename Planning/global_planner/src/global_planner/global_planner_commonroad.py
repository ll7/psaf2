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

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from custom_carla_msgs.msg import GlobalPathLanelets
from tf.transformations import euler_from_quaternion


class GlobalPlanner:
    # TODO: GOAL AS GPS POINT
    def __init__(self, role_name):
        self.role_name = role_name
        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.target_sub = rospy.Subscriber(f"/psaf/{self.role_name}/target_point", NavSatFix, self.target_received)
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_received)
        self.inital_pose_sub = rospy.Subscriber(f"/initialpose", PoseWithCovarianceStamped, self.init_pose_received)

        self.path_pub = rospy.Publisher(f"/psaf/{self.role_name}/global_path", Path, queue_size=1, latch=True)
        self.lanelet_pub = rospy.Publisher(f"/psaf/{self.role_name}/global_path_lanelets", GlobalPathLanelets,
                                           queue_size=1, latch=True)

        self.scenario = None
        self.planning_problem_Set = None
        self.current_pos = None
        self.current_orientation = None

    def map_received(self, msg):
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open()
        self.scenario.scenario_id = "DEU"
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
