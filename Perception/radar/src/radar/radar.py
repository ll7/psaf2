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

class Radar(object):  # pylint: disable=too-few-public-methods
    """
    VehicleController is the combination of two controllers (lateral and longitudinal)
    to perform the low level control a vehicle from client side
    """

    def __init__(self, role_name, target_speed, args_longitudinal=None, args_lateral=None):

        self._current_speed = 0.0
        self.max_dist_to_path = 2 # max distance a point can be from the path to be considered
        self.safety_time = 2.0 #time to wait if no obstacle detected
        self.safety_distance = 100 #distance to publish if no obstacle detected
        self._current_pose = Pose()
        self.current_time = rospy.get_time()
        self.role_name = role_name
        self.path = None        

        self._radar_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/radar/front/radar_points", PointCloud2, self.radar_updated)
        self._dist_publisher = rospy.Publisher(f"psaf/{role_name}/radar/distance", Float64, queue_size=1)
        self._route_subscriber = rospy.Subscriber(
            f"/psaf/{role_name}/global_path", Path, self.route_updated)
        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(role_name), Odometry, self.odometry_updated)
        self._points_publisher = rospy.Publisher(f"psaf/{role_name}/radar/points", PointCloud2, queue_size=1)
        self._slowed_publisher = rospy.Publisher(f"psaf/{role_name}/bt/condition/slowed_by_car_in_front", Bool, queue_size=1)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def odometry_updated(self, odo):
        """
        callback on new odometry data
        """
        self._current_speed = math.sqrt(odo.twist.twist.linear.x ** 2 +
                                        odo.twist.twist.linear.y ** 2 +
                                        odo.twist.twist.linear.z ** 2) * 3.6
        self._current_pose = odo.pose.pose

    def debug_filter_points(self, points):
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = "map"
        cloud_msg.header.stamp = rospy.Time.now()            
        xyz = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in points]                       
        point_cloud = pc2.create_cloud_xyz32(cloud_msg.header, xyz)
        self._points_publisher.publish(point_cloud)  

    def calc_dist(self, points):               
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
        px =[posen.pose.position.x for posen in self.path.poses]
        py =[posen.pose.position.y for posen in self.path.poses]
        points = []        
        for p in poses_transformed:
            dx =[p.pose.position.x - icx for icx in px]
            dy =[p.pose.position.y - icy for icy in py]
            d = np.hypot(dx,dy)           
            dist = min(d)            
            if dist < max_dist_to_path:                    
                points.append(p)
        return points   

    def transform_into_map_coords(self, points):
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
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error in transformation")                       

        return [tf2_geometry_msgs.do_transform_pose(p, trans) for p in poses]

    def radar_updated(self, msg):
        if self.path != None:
            points = pc2.read_points(msg, skip_nans=True, field_names=("x","y","z"))           
            transformed_radar_poses = self.transform_into_map_coords(points)
            points = self.filter_poses(self.max_dist_to_path, transformed_radar_poses)
            self.debug_filter_points(points)
            self.calc_dist(points)
            
    def route_updated(self, path):
        self.path = path

    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass

def main():
    rospy.init_node('radar', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    target_speed = rospy.get_param("~target_speed", 0)
    radar = Radar(role_name, target_speed)
    try:
        radar.run()
    finally:
        del radar
    rospy.loginfo("Done")

if __name__ == "__main__":
    main()



