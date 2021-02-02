import copy
import time

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import rospy


try:
    mpl.use('Qt5Agg')
except ImportError:
    mpl.use('TkAgg')

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.geometry.shape import Rectangle
from commonroad.common.util import AngleInterval, Interval
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType, DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction

from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from SMP.motion_planner.motion_planner import MotionPlanner
from SMP.motion_planner.plot_config import StudentScriptPlotConfig
from commonroad_dc.boundary import boundary
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker, \
    create_collision_object
from SMP.motion_planner.queue import PriorityQueue
from SMP.motion_planner.utility import create_trajectory_from_list_states

from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from derived_object_msgs.msg import ObjectArray

from helper_functions import calc_egocar_yaw, calc_path_yaw

PLOT_CONFIG = StudentScriptPlotConfig(DO_PLOT=False)
PATH_SCENARIO = "./commonroad_map_reduced.xml"
PATH_MOTION_PRIMITIVES = "V_9.0_9.0_Vstep_0_SA_-0.2_0.2_SAstep_0.2_T_0.5_Model_BMW_320i.xml" #'V_10.0_10.0_Vstep_0_SA_-0.2_0.2_SAstep_0.4_T_0.5_Model_BMW320i.xml' #'V_0.0_20.0_Vstep_4.0_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.xml'

# Literatur
# evtl: . Magdici and M. Althoff, “Fail-safe motion planning of autonomousvehicles,” inProc. of the 19th International IEEE Conference onIntelligent Transportation Systems, 2016, pp. 452–458.
# https://mediatum.ub.tum.de/doc/1546126/
# https://mediatum.ub.tum.de/doc/1379612/1379612.pdf
# https://commonroad.in.tum.de/forum/t/desired-usage-of-the-route-planner/332/2
# https://mediatum.ub.tum.de/doc/1379638/776321.pdf

class LocalPlanner():

    def __init__(self, role_name):
        self.role_name = role_name
        self.current_pos = np.zeros(shape=2)
        self.current_orientation = None
        self.current_speed = None
        self.scenario = None
        self.planning_problem_set = None
        self.org_scenario = None
        self.goal_region = None
        self.start_state = None
        self.planning_problem = None
        self.planner = None
        self.static_collision_checker = None
        self.automaton = ManeuverAutomaton.generate_automaton(PATH_MOTION_PRIMITIVES)
        self.shapegroup_triangles_boundary = None
        self.initialize_planner = True
        self.global_path = None

        self.got_objects = True
        self.ego_vehicle_objects = []
        self.obs_counter = 0
        

        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_received)
        self.global_path_sub = rospy.Subscriber(f"/psaf/{self.role_name}/global_path", Path, self.global_path_received)
        self.ego_vehicle_objects_sub = rospy.Subscriber(f"carla/{self.role_name}/objects", ObjectArray, self.ego_vehicle_objects_received)

        self.local_path_pub = rospy.Publisher(f"/psaf/{self.role_name}/local_path", Path, queue_size=1, latch=True)

    def run(self):
        """
        Control loop, calc local path every second.
        :return:
        """
        r = rospy.Rate(2)
        ego_vehicle_objects_before = self.ego_vehicle_objects
        while not rospy.is_shutdown():
            if self.global_path is not None:
                self.calc_route()
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass

    def map_received(self, msg):
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open()
        self.scenario.scenario_id = "DEU"
        self.org_scenario = copy.deepcopy(self.scenario)

    def global_path_received(self, msg):
        self.global_path = msg

    def odometry_received(self, msg):
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.current_orientation = calc_egocar_yaw(msg.pose.pose)

    def ego_vehicle_objects_received(self, msg):
        self.ego_vehicle_objects = []
        self.ego_vehicle_objects = msg.objects

    def calc_route(self):
        start = time.time()

        # get index of next point on route
        px = [poses.pose.position.x for poses in self.global_path.poses]
        py = [poses.pose.position.y for poses in self.global_path.poses]
        dx = [self.current_pos[0] - icx for icx in px]
        dy = [self.current_pos[1] - icy for icy in py]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        self.set_start_state(self.current_pos, self.current_orientation, 50 / 3.6)
        self.set_goal_region(np.array([self.global_path.poses[target_idx + 3000].pose.position.x, self.global_path.poses[target_idx + 3000].pose.position.y]), orientation=calc_path_yaw(self.global_path, target_idx+3000))

        # add obstacles of carla
        obstacles = []
        for i,o in enumerate(self.ego_vehicle_objects):
            speed = np.sqrt( o.twist.linear.x ** 2 + o.twist.linear.y ** 2 + o.twist.linear.z ** 2) - 10/3.6
            obstacles.append(self.create_dynamic_obstacle(self.org_scenario.generate_object_id(), 5, 2.5, o.pose.position.x, o.pose.position.y, calc_egocar_yaw(o.pose), 0, speed, self.scenario.dt))
        
        self.create_planning_problem(obstacles)

        if self.initialize_planner:
            self.init_planner()
            self.initialize_planner = False

        # calc path and publish. If planning takes too long nothing is published.
        route = self.plan(obstacles)
        if route:
            traj = create_trajectory_from_list_states(route)
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = rospy.Time.now()
            for state in traj.state_list:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = state.position[0]
                pose.pose.position.y = state.position[1]
                pose.pose.position.z = 0
                path_msg.poses.append(pose)
           
            self.local_path_pub.publish(path_msg)
            print("Published local path")
        else:
            print("No route calced")

        print(f"Time for local path compution: {time.time() -  start}")

    def set_goal_region(self, position, orientation=None, velocity=0):
        """
        Set the goal/goal region
        :param position: np.array with center position of goal
        :param orientation: yaw - angle. Double or Interval
        :return:
        """
        if orientation is None:
            orientation = AngleInterval(-np.pi, np.pi - 0.000001)
        else:
            orientation = AngleInterval(np.clip(orientation - 0.1, -np.pi, np.pi), np.clip(orientation + 0.1, -np.pi, np.pi))
        velocity = Interval(0, 40/3.6)
        goal_state = State(position=Rectangle(2, 2, center=position), time_step=Interval(1, 200),
                           orientation=orientation, velocity=velocity)
        self.goal_region = GoalRegion([goal_state])

    def set_start_state(self, position, orientation, velocity):
        self.start_state = State(position=position, orientation=orientation, velocity=velocity, time_step=1,
                                 yaw_rate=0.0, slip_angle=0.0)

    def create_planning_problem(self, obstacles=None):
        self.planning_problem = PlanningProblem(10000, self.start_state, self.goal_region)
        if 10000 in self.planning_problem_set.planning_problem_dict.keys():
            del self.planning_problem_set.planning_problem_dict[10000]
        self.planning_problem_set.add_planning_problem(self.planning_problem)
        self.scenario = copy.deepcopy(self.org_scenario)
        if obstacles:
            for i, o in enumerate(obstacles):
                print("added obstacle")
                self.scenario.add_objects(o)

    def init_planner(self):
        self.planner = MotionPlanner.GreedyBestFirstSearch(self.scenario, self.planning_problem, self.automaton,
                                                           PLOT_CONFIG)
        self.static_collision_checker = copy.deepcopy(self.planner.collision_checker)
        _, self.shapegroup_triangles_boundary = boundary.create_road_boundary_obstacle(self.scenario,
                                                                                       method='aligned_triangulation',
                                                                                       axis=2)

    def cut_scenario(self, radius):
        lanelets_to_keep = self.scenario.lanelet_network.lanelets_in_proximity(self.current_pos, radius)
        lanelets_to_remove = []
        new_network = LaneletNetwork()
        for id, l in self.scenario.lanelet_network._lanelets.items():
            if l not in lanelets_to_keep:
                lanelets_to_remove.append(id)
        for l in lanelets_to_remove:
            del self.scenario.lanelet_network._lanelets[l]
        self.scenario.lanelet_network.cleanup_lanelet_references()

    def plot_scenario(self):
        plt.figure(figsize=(8, 8))
        draw_object(self.scenario)
        draw_object(self.planning_problem_set)
        plt.gca().set_aspect('equal')
        plt.margins(0, 0)
        plt.show()
        # close the figure to continue!


    def plan(self, obstacles=None):
        # mostly same as SMP.motion_planner.search_algorithms.base_class.__init__()
        # reset planner, but keep the static collision_map
        # store input parameters
        self.obs_counter = 0
        self.planner.scenario = self.scenario
        self.planner.planningProblem = self.planning_problem
        self.planner.automaton = self.automaton
        self.planner.shape_ego = self.automaton.shape_ego

        # create necessary attributes
        self.planner.lanelet_network = self.scenario.lanelet_network
        self.planner.list_obstacles = self.scenario.obstacles
        self.planner.state_initial = self.planning_problem.initial_state
        self.planner.motion_primitive_initial = self.automaton.create_initial_motion_primitive(self.planning_problem)
        self.planner.list_ids_lanelets_initial = []
        self.planner.list_ids_lanelets_goal = []
        self.planner.time_desired = None
        self.planner.position_desired = None
        self.planner.velocity_desired = None
        self.planner.orientation_desired = None
        self.planner.distance_initial = None
        self.planner.dict_lanelets_costs = {}

        # visualization parameters
        self.planner.config_plot = PLOT_CONFIG
        self.planner.path_fig = None

        # remove unnecessary attributes of the initial state
        if hasattr(self.planner.state_initial, 'yaw_rate'):
            del self.planner.state_initial.yaw_rate

        if hasattr(self.planner.state_initial, 'slip_angle'):
            del self.planner.state_initial.slip_angle

        # parse planning problem
        self.planner.parse_planning_problem()
        self.planner.initialize_lanelets_costs()

        # create new collision_checker as it adds the obstacles in its creation.
        self.planner.collision_checker = create_collision_checker(self.scenario)

        # create a second shapegroup of the map so you don't have to change code of commonroad_search.
        self.planner.collision_checker.add_collision_object(self.shapegroup_triangles_boundary)

        # reset priority_queue of GreedyBestFirstSearch
        self.planner.frontier = PriorityQueue()

        # calc path
        start = time.time()
        path, _, _ = self.planner.execute_search()
        print(f"Plan Only: {time.time() - start}")
        # plt.title("abc")
        # # close the figure to continue
        # plt.show()
        return path

    def create_static_obstacle(self, id, width, length, x, y, orientation, time_step):
        static_obstacle_id = id + self.obs_counter
        self.obs_counter += 1
        static_obstacle_type = ObstacleType.PARKED_VEHICLE
        static_obstacle_shape = Rectangle(width, length)
        static_obstacle_initial_state = State(position=np.array([x, y]), orientation=orientation, time_step=time_step)
        static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape,
                                         static_obstacle_initial_state)
        return static_obstacle

    def create_dynamic_obstacle(self, id, width, length, x, y, orientation, time_step, velocity, dt):
        dynamic_obstacle_initial_state = State(position=np.array([x, y]), velocity=velocity, orientation=orientation,
                                               time_step=time_step)
        state_list = []
        for i in range(1, 41):
            new_position = np.array([dynamic_obstacle_initial_state.position[0] + dt * i * velocity, y])
            new_state = State(position=new_position, velocity=velocity, orientation=orientation, time_step=i)
            state_list.append(new_state)

        dynamic_obstacle_trajectory = Trajectory(1, state_list)
        dynamic_obstacle_shape = Rectangle(width, length)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)
        dynamic_obstacle_id = id + self.obs_counter
        self.obs_counter += 1
        dynamic_obstacle_type = ObstacleType.CAR
        dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape,
                                           dynamic_obstacle_initial_state, dynamic_obstacle_prediction)
        return dynamic_obstacle


if __name__ == "__main__":
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    lp = LocalPlanner(role_name)
    try:
        lp.run()
    finally:
        del lp
