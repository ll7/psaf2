import copy
import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64,String
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Path, Odometry
from helper_functions import next_point_on_path
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose

from commonroad.common.file_reader import CommonRoadFileReader
from custom_carla_msgs.srv import TrafficOnLanelet
#from custom_carla_msgs.msg import GlobalPathLanelets

class Lidar(object):  
    """
    Lidar uses lidar-sensor data and scenario.lanelets to check for traffic on right or left lane
    """

    def __init__(self, role_name):
        self._current_speed = 0.0        
        self._current_pose = Pose()
        self.current_pos = np.zeros(shape=2)

        self.role_name = role_name

        self.max_dist_lidar = 20 # maximum distance of lidar-sensor-data that should be considered

        self.scenario = None 
        self.planning_problem = None  
        self.lanelets_on_route = None

        self.current_lanelet = None
        self.left_lanelet = None
        self.right_lanelet = None     
        self.points = None

        #Subscriber
        self.map_sub = rospy.Subscriber(f"/psaf/{self.role_name}/commonroad_map", String, self.map_received)
        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(role_name), Odometry, self.odometry_updated)   
        self._lidar_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/lidar/lidar1/point_cloud", PointCloud2, self.lidar_updated)

        #Publisher
        self.obstacle_on_left_lane_pub = rospy.Publisher(f"/psaf/{self.role_name}/obstacle_on_left_lane", Float64, queue_size = 1)
        self.obstacle_on_right_lane_pub = rospy.Publisher(f"/psaf/{self.role_name}/obstacle_on_right_lane", Float64, queue_size = 1)
        self._points_publisher = rospy.Publisher(f"psaf/{role_name}/lidar/points", PointCloud2, queue_size=1)
       
        #Necessary to transform into map coordinates
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def map_received(self, msg):
        """Map Update Callback
        """
        self.scenario, self.planning_problem_set = CommonRoadFileReader(msg.data).open()
        self.scenario.scenario_id = "DEU"

    def odometry_updated(self, odo):
        """Odometry Update Callback
        """
        self._current_speed = math.sqrt(odo.twist.twist.linear.x ** 2 +
                                        odo.twist.twist.linear.y ** 2 +
                                        odo.twist.twist.linear.z ** 2) * 3.6
        self._current_pose = odo.pose.pose
        self.current_pos = np.array([odo.pose.pose.position.x, odo.pose.pose.position.y])    

    def lidar_updated(self, msg):
        """Lidar Update Callback
        """
        self.points = pc2.read_points(msg, skip_nans=True, field_names=("x","y","z"))

    def debug_filter_points(self, points):
        """Creates a PointCloud2 Object from array of Poses that is then published for debug purposes
        Args:
            points ([type]): array of poses in map frame
        """
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = "map"
        cloud_msg.header.stamp = rospy.Time.now()            
        xyz = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in points]                       
        point_cloud = pc2.create_cloud_xyz32(cloud_msg.header, xyz)
        self._points_publisher.publish(point_cloud)  

    def transform_lidar_into_map_coords(self, points):
        """transforms a list of points from ego_vehicle/lidar/lidar1/ frame to map frame using tf2 and the current transformation
        Args:
            points ([type]): points in ego_vehicle/lidar/lidar1/ frame
        Returns:
            [type]: points in map frame
        """
        poses = []
        for p in points:
            pose = PoseStamped()
            pose.header.frame_id = "ego_vehicle/lidar/lidar1"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            poses.append(pose)

        try:
            trans = self.tf_buffer.lookup_transform('map', 'ego_vehicle/lidar/lidar1', rospy.Time())
            return [tf2_geometry_msgs.do_transform_pose(p, trans) for p in poses]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Error in transformation")                       


    def calc_dist(self, points):
        """calculates distance of the closest point in points and returns its distance
        Args:
            points ([type]): array of poses in map frame
        """                    
        dist_x = [self._current_pose.position.x - p.pose.position.x for p in points]
        dist_y = [self._current_pose.position.y - p.pose.position.y for p in points]
        dist = np.hypot(dist_x,dist_y) 
        if len(dist) > 0:
            return min(dist)           
        else:                
            return 0               
    
    def filter_lidar_poses(self, lanelet, transformed_lidar_poses):
        """filters a list of poses by their distance to the path
        Args:
            lanelet (Lanelet): lanelet which needs to checked for obstacles
            poses_transformed ([type]): list of poses to consider
        Returns:
            [type]: filtered list of points
        """
        points = []                
        if transformed_lidar_poses is None:
            return points
        for p in transformed_lidar_poses:
            dx = [p.pose.position.x - x for x in lanelet.center_vertices[:,0]]
            dy = [p.pose.position.y - y for y in lanelet.center_vertices[:,1]]            
            d = np.hypot(dx,dy)
            if d.any():   
                dist = min(d)           
                if dist < 2: #lane_width/2      
                    points.append(p)
        return points


    def main_loop(self):
        """Main Loop to check if there are any obstacles on left or right lane
        """
        if self.points is None:
            return
        points = self.points
        transformed_lidar_poses = self.transform_lidar_into_map_coords(points)        
        self.get_right_and_left_lanelet()        
        if self.left_lanelet is not None:            
            filtered_poses_left = self.filter_lidar_poses(self.left_lanelet, transformed_lidar_poses)           
            if len(filtered_poses_left) > 0:                 
                dist = self.calc_dist(filtered_poses_left)           
                if dist > 0 and dist < self.max_dist_lidar:        
                    self.obstacle_on_left_lane_pub.publish(dist) 
                else:
                    self.obstacle_on_left_lane_pub.publish(np.inf) 
            # if there are no points on left_lane, checks successor and predecessor
            else:
                filtered_poses_left = self.filter_lidar_poses(self.scenario.lanelet_network.find_lanelet_by_id(self.left_lanelet.successor[0]), transformed_lidar_poses)

                if len(filtered_poses_left) > 0:                 
                    dist = self.calc_dist(filtered_poses_left)           
                    if dist > 0 and dist < self.max_dist_lidar:        
                        self.obstacle_on_left_lane_pub.publish(dist) 
                    else:
                        self.obstacle_on_left_lane_pub.publish(np.inf)
                else:
                    filtered_poses_left = self.filter_lidar_poses(self.scenario.lanelet_network.find_lanelet_by_id(self.left_lanelet.predecessor[0]), transformed_lidar_poses)

                    if len(filtered_poses_left) > 0:                 
                        dist = self.calc_dist(filtered_poses_left)           
                        if dist > 0 and dist < self.max_dist_lidar:        
                            self.obstacle_on_left_lane_pub.publish(dist) 
                        else:
                            self.obstacle_on_left_lane_pub.publish(np.inf)
                    else:
                        self.obstacle_on_right_lane_pub.publish(np.inf) 
                
        if self.right_lanelet is not None:            
            filtered_poses_right = self.filter_lidar_poses(self.right_lanelet, transformed_lidar_poses)
            if len(filtered_poses_right) > 0:                
                dist = self.calc_dist(filtered_poses_right)               
                if dist > 0 and dist < self.max_dist_lidar:  
                    self.obstacle_on_right_lane_pub.publish(dist) 
                else:
                    self.obstacle_on_right_lane_pub.publish(np.inf)
             # if there are no points on right_lane, checks successor and predecessor  
            else:
                filtered_poses_right = self.filter_lidar_poses(self.scenario.lanelet_network.find_lanelet_by_id(self.right_lanelet.successor[0]), transformed_lidar_poses)

                if len(filtered_poses_right) > 0:                 
                    dist = self.calc_dist(filtered_poses_right)           
                    if dist > 0 and dist < self.max_dist_lidar:        
                        self.obstacle_on_right_lane_pub.publish(dist) 
                    else:
                        self.obstacle_on_right_lane_pub.publish(np.inf)
                else:
                    filtered_poses_right = self.filter_lidar_poses(self.scenario.lanelet_network.find_lanelet_by_id(self.right_lanelet.predecessor[0]), transformed_lidar_poses)

                    if len(filtered_poses_right) > 0:                 
                        dist = self.calc_dist(filtered_poses_right)           
                        if dist > 0 and dist < self.max_dist_lidar:        
                            self.obstacle_on_right_lane_pub.publish(dist) 
                        else:
                            self.obstacle_on_right_lane_pub.publish(np.inf)    
                    else:
                        self.obstacle_on_right_lane_pub.publish(np.inf)      
       
    
    def get_right_and_left_lanelet(self):
        """Gets right and left Lanelet of current_lanelet
        """    
        if self.scenario is not None:
            possible_lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position([np.array(list(self.current_pos))])[0]
            self.current_lanelet = None
            self.right_lanelet = None
            self.left_lanelet = None            
            for lane_id in possible_lanelet_ids:  
                self.current_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lane_id)     
                if self.current_lanelet is not None:
                    if self.current_lanelet.adj_left is not None:
                        self.left_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(self.current_lanelet.adj_left)
                    if self.current_lanelet.adj_right is not None:
                        self.right_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(self.current_lanelet.adj_right)

                     
    def check_lanelet_free(self, req):
        """
        Check_lanelet_free. The function is registered as a ROS-Service. The service definition file is located in
        psaf/Interfaces/msgs/srv/TrafficOnLanelet.srv. Possible keywords are:
        None: no lanelet to check.
        isRoundabout: lanelet to check is in roundabout
        lanlet_id: lanlet-id
        :param req: ROS srv message.
        :return: obstacle on lanelet (bool)        
        """
        lanelet_id = req.lanelet_id
        if lanelet_id != 0:                    
            lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            if self.points is None:              
                return False             
            points = list(self.points)
            if len(points) == 0:
                return False 
            transformed_lidar_poses = self.transform_lidar_into_map_coords(points)    
            if lanelet is not None:  
                filtered_poses = self.filter_lidar_poses(lanelet, transformed_lidar_poses)      
                if len(filtered_poses) > 0:                 
                    dist = self.calc_dist(filtered_poses)           
                    if dist > 0 and dist < self.max_dist_lidar:
                        return False 
                    else:
                        return True 
                else:
                    # if there are no points on lanelet, checks successor
                    filtered_poses = self.filter_lidar_poses(self.scenario.lanelet_network.find_lanelet_by_id(lanelet.successor[0]), transformed_lidar_poses)
                    if len(filtered_poses) > 0:                 
                        dist = self.calc_dist(filtered_poses)           
                        if dist > 0 and dist < self.max_dist_lidar: 
                            return False                     
                        else:
                            return True
                    else:
                        # if there are no points on lanelet and lanelet.successor, checks predecessor  
                        filtered_poses = self.filter_lidar_poses(self.scenario.lanelet_network.find_lanelet_by_id(lanelet.predecessor[0]), transformed_lidar_poses)
                        if len(filtered_poses) > 0:                 
                            dist = self.calc_dist(filtered_poses)           
                            if dist > 0 and dist < self.max_dist_lidar:  
                                return False
                            else:
                                return True
        return True     
                    
    
    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
                self.main_loop()
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass
        

def main():
    rospy.init_node('lidar', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")    
    lidar = Lidar(role_name)
    s = rospy.Service("check_lanelet_free", TrafficOnLanelet, lidar.check_lanelet_free)
    try:
        lidar.run()
    finally:
        del lidar
    rospy.loginfo("Done")

if __name__ == "__main__":
    main()
    



