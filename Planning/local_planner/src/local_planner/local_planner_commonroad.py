import copy
import numpy as np
import rospy
import json

from commonroad.common.file_reader import CommonRoadFileReader

from std_msgs.msg import String, Int32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from custom_carla_msgs.msg import GlobalPathLanelets, NextLanelet
from custom_carla_msgs.srv import UpdateLocalPath


class LocalPlanner:
    def __init__(self, role_name):
        # initialize all class variables
        self.role_name = role_name
        self.current_pos = np.zeros(shape=2)
        self.scenario = None

        self.lanelets_path_ids = None
        self.lanelets_roundabout_outgoing = None
        self.lanelets_roundabout_inside = None
        self.lanelets_roundabout_inside_outer_circle = None

        # adjacent_lanelets = [[1], [2, 3], [4, 5], [6]] 2D-List that creates the global path. If more than one element
        # exists in a sub-list it means that there is a adjacent lane. A possible path would be: 1,2,4,6 or 1,3,4,6
        self.adjacent_lanelets = None
        self.adjacent_lanelets_flattened = None  # list of all possible lanelets on route

        # add ros subscribers
        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self.odometry_sub = rospy.Subscriber(f"carla/{self.role_name}/odometry", Odometry, self.odometry_received)
        self.lanelet_sub = rospy.Subscriber(f"/psaf/{self.role_name}/global_path_lanelets", GlobalPathLanelets, self.lanelets_received)
        self.roundabout_pub = rospy.Publisher(f"/psaf/{self.role_name}/distance_next_roundabout", NextLanelet, queue_size=1)
        self.roundabout_exit_pub = rospy.Publisher(f"/psaf/{self.role_name}/distance_exit_roundabout", Point, queue_size=1, latch=True)

        # add ros publishers
        self.local_path_pub = rospy.Publisher(f"/psaf/{self.role_name}/local_path", Path, queue_size=1, latch=True)
        self.first_lanelet_roundabout = rospy.Publisher(f"/psaf/{self.role_name}/first_lanelet_roundabout", Int32 , queue_size=1, latch=True)

    def map_received(self, msg):
        self.scenario, _ = CommonRoadFileReader(msg.data).open()
        self.scenario.scenario_id = "DEU"
        self.org_scenario = copy.deepcopy(self.scenario)

    def lanelets_received(self, msg):
        self.lanelets_path_ids = msg.lanelet_ids
        self.lanelets_roundabout_inside = msg.lanelet_ids_roundabout_inside
        self.lanelets_roundabout_outgoing = msg.lanelet_ids_roundabout_outgoing
        self.lanelets_roundabout_inside_outer_circle = msg.lanelet_ids_roundabout_inside_outer_circle

        self.adjacent_lanelets = json.loads(msg.adjacent_lanelet_ids)
        self.adjacent_lanelets_flattened = [item for sublist in self.adjacent_lanelets for item in sublist]

    def odometry_received(self, msg):
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def _change_lane(self, current_lanelet, idx_nearest_point, left=False, right=False):
        """
        Plan a lane change to left/right lanelet.
        :param current_lanelet: Lanelet object which the ego vehicle is located on.
        :param idx_nearest_point: Nearest vertex point on the lanelet.
        :param left: Change to left lane.
        :param right: Change to right lane.
        :return: Path for lane change.
        """
        if (left and right) or (not left and not right):
            raise ValueError("One of left and right must be true")
        path = []
        # add next 10 center_vetices of current lanelet
        path.extend(current_lanelet.center_vertices[idx_nearest_point:idx_nearest_point + 10])
        # get the adjacent lanelet
        if left:
            rospy.loginfo("Lane change left")
            adj_lane = self.scenario.lanelet_network.find_lanelet_by_id(current_lanelet.adj_left)
        if right:
            rospy.loginfo("Lane change right")
            adj_lane = self.scenario.lanelet_network.find_lanelet_by_id(current_lanelet.adj_right)
        # add vertecies of adjacent lanelets to path. Leave a gap for smooth change.
        path.extend(adj_lane.center_vertices[idx_nearest_point + 30:])
        return path

    # Python program to illustrate the intersection 
    # of two lists using set() method 
    def intersection(self, lst1, lst2): 
        return list(set(lst1) & set(lst2))

    def intersection_id_of_two_center_vertices(self, lane_one, lane_two):
        minimum = 100
        idx_minimum_one = 0
        idx_minimum_two = 0
        for i,p in enumerate(lane_two.center_vertices):
            distances = np.linalg.norm(lane_one.center_vertices - p, axis = 1)
            if np.min(distances) < minimum:
                minimum = np.min(distances)
                idx_minimum_one = np.argmin(distances)
                idx_minimum_two = i
        return minimum, idx_minimum_one, idx_minimum_two

    def update_local_path(self, req):
        """
        Update local path. The function is registered as a ROS-Service. The service definition file is located in
        psaf/Interfaces/msgs/srv/UpdateLocalPath.srv. Possible keywords are:
        None: Drive straight on current lanelet.
        approach_intersection: Plan a path for the approach and the behaviour in a intersection. Changes to the correct
         lane for a turn if necessary
        leave_intersection: Plan a path to leave the intersection and drive straight in the next lanelet.
        approach roundabout:  
        change_lane_left: Plan a path for a left lane change.
        change_lane_right: Plan a path for a right lane change.
        :param req: ROS srv message.
        :return: Created path for next two lanelets.
        """
        # resolve ros srv message
        change_lane_left = req.change_lane_left
        change_lane_right = req.change_lane_right
        

        approach_roundabout = req.approach_roundabout
        approach_intersection = req.approach_intersection
        leave_intersection = req.leave_intersection
        path = []
        # get all possible lanelet ids of current position.
        # In a intersection it is possible that you get more than one lanelet.
        if self.scenario is None:
            return
        possible_lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position([np.array(list(self.current_pos))])[0]
        # Use the first lanelet that is also on the previously calculated global path.
        for lane_id in possible_lanelet_ids:
            if lane_id in self.adjacent_lanelets_flattened:
                current_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lane_id)
                distances_to_center_vertices = np.linalg.norm(current_lanelet.center_vertices - self.current_pos,
                                                              axis=1)
                idx_nearest_point = np.argmin(distances_to_center_vertices)  # get nearest vertex in current lanelet
                if change_lane_left:
                    path.extend(self._change_lane(current_lanelet, idx_nearest_point, left=True))
                elif change_lane_right:
                    path.extend(self._change_lane(current_lanelet, idx_nearest_point, right=True))
                elif approach_roundabout:
                    rospy.loginfo("Approach Roundabout")
                    path.extend(current_lanelet.center_vertices[idx_nearest_point:])
                    
                    #über globalen Pfad den Ausgang finden
                    if self.lanelets_roundabout_outgoing is not None:
                        outgoing_lanes = (self.intersection(list(self.lanelets_roundabout_outgoing), list(self.lanelets_path_ids)))
                        #rospy.loginfo(f"number outgoing_lanes = {len(outgoing_lanes)}" )
                        #rospy.loginfo(f"outgoing_lanes_id = {outgoing_lanes[0]}")
                        
                        outgoing_lane = self.scenario.lanelet_network.find_lanelet_by_id(outgoing_lanes[0])
                        if outgoing_lane is None:
                            rospy.loginfo("No outgoing lane found!")
                            return
                        #distanzen zu outer circle lanelets
                        distances_to_outer_circle = []
                        closest_lanelet_on_outer_circle = None
                        for inner_lanelet_id in self.lanelets_roundabout_inside_outer_circle:
                            inner_lane = self.scenario.lanelet_network.find_lanelet_by_id(inner_lanelet_id)
                            distances_to_right_vertices = np.linalg.norm(inner_lane.right_vertices - current_lanelet.center_vertices[-1], axis=1)

                            if len(distances_to_right_vertices) > 0:
                                if len(distances_to_outer_circle) > 0:
                                    if np.min(distances_to_right_vertices) < np.min(distances_to_outer_circle):
                                        idx_nearest_point = np.argmin(distances_to_right_vertices) 
                                        closest_lanelet_on_outer_circle = inner_lane                                            
                                else:
                                    idx_nearest_point = np.argmin(distances_to_right_vertices) 
                                    closest_lanelet_on_outer_circle = inner_lane 
                                distances_to_outer_circle.append(np.min(distances_to_right_vertices)) 
                        #rospy.loginfo(distances_to_outer_circle)           
                        #sobald eine distanz < 1, wechsel zu nächstem punkt auf mittellinie dieser outer circle lanelet
                                    
                        if len(distances_to_outer_circle) > 0:
                            if np.min(distances_to_outer_circle) < 20:
                                rospy.loginfo("distance to outer circle < 20")
                                if closest_lanelet_on_outer_circle is not None:                                    
                                    #Schnittpunkt Kreisverkehrspur und aktuelle spur finden
                                    minimum, idx_minimum_one, idx_minimum_two = self.intersection_id_of_two_center_vertices(closest_lanelet_on_outer_circle, current_lanelet)
                                    if minimum > 2:
                                        successor_id = current_lanelet.successor[0]
                                        if successor_id is not None:
                                            successor = self.scenario.lanelet_network.find_lanelet_by_id(successor_id)
                                            path.extend(successor.center_vertices)
                                            minimim_succesor, idx_minimum_succesor, idx_minimum_closest = self.intersection_id_of_two_center_vertices(successor, closest_lanelet_on_outer_circle)
                                            path = path[:-(len(successor.center_vertices) - idx_minimum_succesor + 2)]
                                            path.extend(closest_lanelet_on_outer_circle.center_vertices[idx_minimum_closest + 2:])                                        
                                    else:
                                        path = path[:-(len(current_lanelet.center_vertices) - idx_minimum_two + 2)]
                                        path.extend(closest_lanelet_on_outer_circle.center_vertices[idx_minimum_one + 2:])


                                    self.first_lanelet_roundabout.publish(closest_lanelet_on_outer_circle.lanelet_id)
                                    #solange successor suchen, bis abstand zu outgoing < 4
                                    #außen im Kreisverkehr bis Ausgang fahren
                                    while closest_lanelet_on_outer_circle is not None:
                                        #rospy.loginfo("Extend path with next lane")
                                        minimum, idx_minimum_outgoing, idx_minimum_closest_lanelet = self.intersection_id_of_two_center_vertices(outgoing_lane, closest_lanelet_on_outer_circle)
                                        #vergleichen mit closest_lanelet_on_outer_circle.center_vertices
                                        distances_to_outgoing_center_vertices = [np.linalg.norm(outgoing_lane.center_vertices - p, axis=1) for p in closest_lanelet_on_outer_circle.center_vertices]
                                        # distances_to_outgoing_center_vertices = np.linalg.norm(outgoing_lane.center_vertices - closest_lanelet_on_outer_circle.center_vertices[-1], axis=1)
                                        if len(distances_to_outgoing_center_vertices) > 0:
                                            if np.min(minimum) < 2:
                                                #rospy.loginfo(f"next Lane is outgoing_lane with distance = {minimum}")
                                                less_steps = (len(closest_lanelet_on_outer_circle.center_vertices) - idx_minimum_closest_lanelet)
                                                #rospy.loginfo(f"reduced path with length {len(closest_lanelet_on_outer_circle.center_vertices)} by {less_steps}")
                                                if len(path) > less_steps + 3:
                                                    path = path[:-(less_steps + 3)]
                                                else: 
                                                    path = path[:-less_steps]                                               
                                                if len(outgoing_lane.center_vertices) > idx_minimum_outgoing + 3:
                                                    path.extend(outgoing_lane.center_vertices[(idx_minimum_outgoing + 3):])
                                                else: 
                                                    path.extend(outgoing_lane.center_vertices[idx_minimum_outgoing:])
                                                closest_lanelet_on_outer_circle = None
                                                self.roundabout_exit_pub.publish(Point(outgoing_lane.center_vertices[-1][0], outgoing_lane.center_vertices[-1][1], 0))
                                            else:
                                                #rospy.loginfo(np.min(distances_to_outgoing_center_vertices))
                                                successor = self.intersection(list(closest_lanelet_on_outer_circle.successor), list(self.lanelets_roundabout_inside_outer_circle))[0]
                                                if successor is not None:
                                                    #rospy.loginfo(f"next lane is 1. successor with id {successor}")                                                 
                                                    closest_lanelet_on_outer_circle = self.scenario.lanelet_network.find_lanelet_by_id(successor)
                                                    path.extend(closest_lanelet_on_outer_circle.center_vertices[:])
                                                else:
                                                    #rospy.loginfo("no succesor found")
                                                    closest_lanelet_on_outer_circle = None
                                        else:
                                            closest_lanelet_on_outer_circle = None     
                    
                elif approach_intersection:
                    rospy.loginfo("Approach Intersection")
                    for idx, sector in enumerate(self.adjacent_lanelets):  # find lane_id to get successor
                        if lane_id in sector:
                            break
                    try:
                        next_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(
                            self.adjacent_lanelets[idx + 1][0])  # get successor lanelet
                    except IndexError:
                        rospy.loginfo("Local Planner Intersection Approaching: No successor found")
                        return True
                    if next_lanelet.predecessor[0] == lane_id:  # no lane change
                        rospy.loginfo("Keep Lane")
                        path.extend(current_lanelet.center_vertices[idx_nearest_point:])
                    elif next_lanelet.predecessor[0] == current_lanelet.adj_left:  # lane change left
                        path.extend(self._change_lane(current_lanelet, idx_nearest_point, left=True))
                    elif next_lanelet.predecessor[0] == current_lanelet.adj_right:  # lane change right
                        path.extend(self._change_lane(current_lanelet, idx_nearest_point, right=True))
                    path.extend(next_lanelet.center_vertices[:])  # add center vertices of intersection lanelet
                elif leave_intersection:
                    rospy.loginfo("Leave Intersection")
                    for successor in current_lanelet.successor:
                        if successor in self.adjacent_lanelets_flattened:
                            rospy.loginfo(f"successor lanlet: {successor}")
                            next_lanelet_id = successor
                            next_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(next_lanelet_id)
                            path.extend(current_lanelet.center_vertices[idx_nearest_point:])
                            path.extend(next_lanelet.center_vertices[:])
                            rospy.loginfo("Local Planner Intersection Leaving: Successor found")
                            break
                else:
                    path.extend(current_lanelet.center_vertices[idx_nearest_point:])

        # create ros path message
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
    s = rospy.Service("update_local_path", UpdateLocalPath, lp.update_local_path)  # register ROS-Service
    rospy.spin()  # Node is entirely based on callbacks.
