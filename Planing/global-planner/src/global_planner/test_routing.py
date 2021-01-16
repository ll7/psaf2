import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
import os
from SMP.route_planner.route_planner.route_planner import RoutePlanner
from SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path, \
    get_plot_limits_from_routes

file_path = "/home/brandlju/Desktop/Town2.xml"
#file_path = os.path.join(os.getcwd(), 'scenarios/NGSIM/Lankershim/USA_Lanker-1_1_T-1.xml')
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

plt.figure(figsize=(25, 10))
draw_object(scenario)
draw_object(planning_problem_set)

planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
 # instantiate a route planner
route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)

# plan routes, and save the found routes in a route candidate holder
candidate_holder = route_planner.plan_routes()

# we retrieve the first route in the list
# this is equivalent to: route = list_routes[0]
route = candidate_holder.retrieve_first_route()

print(route._generate_reference_path())
 # retrieve plot limits for better visualization. 
# option 1: plot limits from reference path
# plot_limits = get_plot_limits_from_reference_path(route)
# option 2: plot limits from lanelets in the route
plot_limits = get_plot_limits_from_routes(route)

# determine the figure size for better visualization
size_x = 6
ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
fig = plt.figure(figsize=(size_x, size_x / ratio_x_y))
fig.gca().axis('equal')

draw_route(route, draw_route_lanelets=True, draw_reference_path=False, plot_limits=plot_limits)

# determine the figure size for better visualization
size_x = 6
ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
fig = plt.figure(figsize=(size_x, size_x / ratio_x_y))
fig.gca().axis('equal')

draw_route(route, draw_route_lanelets=True, draw_reference_path=True, plot_limits=plot_limits)

plt.gca().set_aspect('equal')
plt.show()
