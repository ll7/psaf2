import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, Bool
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Path, Odometry
from helper_functions import next_point_on_path
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose

class Radar(object):
    """ Determines distance to car in front
        to improve performance in corners the projected Path of the ego_vehicle is also considered
    """
    def __init__(self, role_name):
        self.max_dist_to_path = 2 # max distance a point can be from the path to be considered
        self.safety_time = 2.0 #time to wait if no obstacle detected
        self.safety_distance = 100 #distance to publish if no obstacle detected
        self._current_pose = Pose()
        self.current_time = rospy.get_time()
        self.role_name = role_name
        self.path = Path()

        self._radar_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/radar/front/radar_points", PointCloud2, self.radar_updated)
        self._route_subscriber = rospy.Subscriber(
            f"/psaf/{role_name}/local_path", Path, self.route_updated)
        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(role_name), Odometry, self.odometry_updated)
        self._points_publisher = rospy.Publisher(
            f"psaf/{role_name}/radar/points", PointCloud2, queue_size=1)
        self._dist_publisher = rospy.Publisher(
            f"psaf/{role_name}/radar/distance", Float64, queue_size=1)
        self._slowed_publisher = rospy.Publisher(
            f"psaf/{role_name}/bt/condition/slowed_by_car_in_front", Bool, queue_size=1)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def odometry_updated(self, odo):
        """Odometry Update Callback
        """
        self._current_pose = odo.pose.pose

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

    def calc_dist(self, points):
        """calculates distance of the closest point in points and publishes its distance
        if no points are passed for safety_time it publishes safety_distance
        Args:
            points ([type]): array of poses in map frame
        """
        dist_x = [self._current_pose.position.x - p.pose.position.x for p in points]
        dist_y = [self._current_pose.position.y - p.pose.position.y for p in points]
        dist = np.hypot(dist_x,dist_y) 
        if len(dist) > 0:
            self._dist_publisher.publish(min(dist))
            self._slowed_publisher.publish(True)
            self.current_time = rospy.get_time()
        else:                
            if rospy.get_time() > self.current_time + self.safety_time:
                self._dist_publisher.publish(self.safety_distance)
                self._slowed_publisher.publish(False)   
                self.current_time = rospy.get_time() 

    def filter_poses(self, max_dist_to_path, poses_transformed):
        """filters a list of poses by their distance to the path
        Args:
            max_dist_to_path (Float): maximum distance a point can be from the path to be included
            poses_transformed ([type]): list of poses to consider
        Returns:
            [type]: filtered list of points
        """
        px =[posen.pose.position.x for posen in self.path.poses]
        py =[posen.pose.position.y for posen in self.path.poses]
        points = []
        for p in poses_transformed:
            dx =[p.pose.position.x - icx for icx in px]
            dy =[p.pose.position.y - icy for icy in py]
            d = np.hypot(dx,dy)
            if d.any():
                dist = min(d)
                if dist < max_dist_to_path:
                    points.append(p)
        return points   

    def transform_into_map_coords(self, points):
        """transforms a list of points from ego_vehicle/radar/front frame to map frame using tf2 and the current transformation
        Args:
            points ([type]): points in ego_vehicle/radar/front frame
        Returns:
            [type]: points in map frame
        """
        poses = []
        for p in points:
            pose = PoseStamped()
            pose.header.frame_id = "ego_vehicle/radar/front"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            poses.append(pose)

        try:
            trans = self.tf_buffer.lookup_transform('map', 'ego_vehicle/radar/front', rospy.Time())
            return [tf2_geometry_msgs.do_transform_pose(p, trans) for p in poses]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error in transformation")                       

        

    def radar_updated(self, msg):
        """
        Callback on Radar data, that performs the processing pipeline
        """
        if self.path != None:
            points = pc2.read_points(msg, skip_nans=True, field_names=("x","y","z"))           
            transformed_radar_poses = self.transform_into_map_coords(points)
            points = self.filter_poses(self.max_dist_to_path, transformed_radar_poses)
            self.debug_filter_points(points)
            self.calc_dist(points)
            
    def route_updated(self, path):
        self.path = path



def main():
    rospy.init_node('radar', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    radar = Radar(role_name)
    # node is based on callbacks
    rospy.spin()

if __name__ == "__main__":
    main()
