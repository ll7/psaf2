import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np

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

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import time
class GlobalPlanner:
    def __init__(self, role_name):
        self.role_name = role_name
        print("_______________________________")
        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.target_sub = rospy.Subscriber("/psaf/target_point", NavSatFix, self.target_received)
        self.odometry_sub = rospy.Subscriber("carla/ego_vehicle/odometry", Odometry, self.odometry_received)

        self.path_pub = rospy.Publisher(f"/psaf/{self.role_name}/global_path", Path, queue_size=1, latch=True)

        self.scenario = None
        self.planning_problem_Set = None
        self.current_pos = None
        self.current_orientation = None

    def map_received(self, msg):
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open() 
        self.scenario.scenario_id = "DEU"
        self.create_global_plan()
        

    def target_received(self, msg):
        print("Target Received")
        self.target_gnss = msg

    def init_pose_received(self, pose):
        print("New Start")
        while self.start_gnss is None or self.lanelet_map is None or self.odometry is None:
            rospy.sleep(0.05)

        self.create_global_plan()

    def odometry_received(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_orientation = msg.pose.pose
        # if self.scenario is None:
        #     return
        # else:     
        #     self.create_global_plan()
    
    def create_global_plan(self):
        start = time.time()
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
        #print(f"Time: {time.time() -start}")

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for p in route.reference_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        #plotting
        plt.cla()
        plt.clf()
        plt.ion()
        # determine the figure size for better visualization
        plot_limits = get_plot_limits_from_routes(route)        
        size_x = 6
        ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
        plt.gca().axis('equal')
        draw_route(route, draw_route_lanelets=True, draw_reference_path=True, plot_limits=plot_limits)
        plt.pause(0.001)



if __name__ == "__main__":
    print("HÄ")
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    gp = GlobalPlanner(role_name)

    # idle so topic is still present
    # (resp. if world changes)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass