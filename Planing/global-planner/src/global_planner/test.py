import matplotlib.pyplot as plt
import utm
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
import os
from SMP.route_planner.route_planner.route_planner import RoutePlanner
from SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path, \
    get_plot_limits_from_routes
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.trajectory import State
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle
import lanelet2
import numpy as np

file_path = "/home/brandlju/Desktop/Town2.xml"

scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

print(scenario.location.geo_transformation)

print(list(planning_problem_set.planning_problem_dict.values())[0].initial_state)

projection = lanelet2.projection.UtmProjector(lanelet2.io.Origin(0, 0))
p = projection.forward(
            lanelet2.core.GPSPoint(-0.0016077391132190087, -8.324308136330368e-05))
x = p.x
y = p.y

rect = Rectangle(2, 2, center=np.array([x, y]))
goal_state = State(position=Rectangle(2, 2, center=np.array([x, y])), time_step=Interval(1, 200), velocity=Interval(0, 10), orientation=AngleInterval(-0.2, 0.2))
start_state = State(position=np.array([0, 200]), time_step=1, velocity=0.0, yaw_rate=0.0, slip_angle=0.0, orientation=0.0)
goal_region = GoalRegion([goal_state])
# goal_region.is_reached(start_state)
p_prob = PlanningProblem(10000, start_state, goal_region)
planning_problem_set.add_planning_problem(p_prob)

plt.figure(figsize=(25, 10))
draw_object(scenario)
draw_object(planning_problem_set)


plt.show()