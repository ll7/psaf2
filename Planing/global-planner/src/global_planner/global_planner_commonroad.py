print(help('modules'))

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from HelpterFunctions import calc_egocar_yaw
from helper_functions.SMP.route_planner.route_planner.route_planner import RoutePlanner
from helper_functions.SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path, \
    get_plot_limits_from_routes
print("Imported")
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.trajectory import State
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class GlobalPlanner:
    def __init__(self, role_name):
        self.role_name = role_name
        print("_______________________________")
        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.target_sub = rospy.Subscriber("/psaf/target_point", NavSatFix, self.target_received)
        self.odometry_sub = rospy.Subscriber("carla/ego_vehicle/odometry", Odometry, self.odometry_received)


        self.scenario = None
        self.planning_problem_Set = None
        self.current_pos = None
        # self.odometry = None


    def map_received(self, msg):
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open() 
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
        # TODO: ADD orientation
    
    def create_global_plan(self):
        goal_state = State(position=Rectangle(2, 2, center=np.array([x, y])), time_step=Interval(1, 200), velocity=Interval(0, 10), orientation=AngleInterval(-0.2, 0.2))
        goal_region = GoalRegion([goal_state])
        start_state = State(position=np.array([x, y]), time_step=1, velocity=0.0, yaw_rate=0.0, slip_angle=0.0, orientation=0.0)
        p_prob = PlanningProblem(10000, start_state, goal_region)
        planning_problem_set.add_planning_problem(p_prob)
        plt.figure(figsize=(25, 10))
        draw_object(self.scenario)
        draw_object(self.planning_problem_set)
        plt.show()
       

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