
import math
import rospy
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

from helper_functions import calc_egocar_yaw, normalize_angle, calc_path_yaw

class StanleyLateralController(object):  # pylint: disable=too-few-public-methods
    """
    StanleyLateralController implements longitudinal control using a PID.
    """

    def __init__(self, k=.5, Kp=1.0, L=2.9, max_steer=30.0):

        self.k = k  # control gain
        self.Kp = Kp  # speed proportional gain
        self.L = L  # [m] Wheel base of vehicle
        self.max_steer = np.deg2rad(max_steer)
        #self.targetpointpublisher = rospy.Publisher("/debug/stanley/targetpoint", PoseStamped, queue_size=1)

    def run_step(self, currentPath, currentPose, currentSpeed):
        current_target_idx, error_front_axle = self.calc_target_index(currentPath, currentPose)
        # theta_e corrects the heading error
        theta_e = normalize_angle(calc_path_yaw(currentPath, current_target_idx) - calc_egocar_yaw(currentPose))

        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.k * error_front_axle, currentSpeed)
        # Steering control      
        delta = theta_e + theta_d
        #if len(currentPath.poses) != 0:
       #    self.targetpointpublisher.publish(currentPath.poses[current_target_idx])
        return np.clip(delta, -self.max_steer, self.max_steer)

    def calc_target_index(self, currentPath, currentPose):
        """
        Compute index in the trajectory list of the target.
        """

        if len(currentPath.poses) == 0:
            return 0, 0
        
        # Calc front axle position
        yaw = calc_egocar_yaw(currentPose)
        fx = currentPose.position.x + self.L * np.cos(yaw)
        fy = currentPose.position.y + self.L * np.sin(yaw)

        # Search nearest point index
        px = [posen.pose.position.x for posen in currentPath.poses]
        py = [posen.pose.position.y for posen in currentPath.poses]
        dx = [fx - icx for icx in px]
        dy = [fy - icy for icy in py]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(yaw + np.pi / 2),
                        -np.sin(yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
    



 # sudo docker run -it --network="host" -e CARLAVIZ_HOST_IP=localhost -e CARLA_SERVER_IP=localhost -e CARLA_SERVER_PORT=2000 mjxu96/carlaviz:latest
