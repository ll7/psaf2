
import math
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

class StanleyLateralController(object):  # pylint: disable=too-few-public-methods
    """
    StanleyLateralController implements longitudinal control using a PID.
    """

    def __init__(self, k=.5, Kp=1.0, L=2.9, max_steer=30.0):

        self.k = k  # control gain
        self.Kp = Kp  # speed proportional gain
        self.L = L  # [m] Wheel base of vehicle
        self.max_steer = np.deg2rad(max_steer)

    def run_step(self, currentPath, currentPose, currentSpeed):
        current_target_idx, error_front_axle = self.calc_target_index(currentPath, currentPose)
        # theta_e corrects the heading error
        theta_e = self.normalize_angle(self.calc_path_yaw(currentPath, current_target_idx) - self.calc_egocar_yaw(currentPose))
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.k * error_front_axle, currentSpeed)
        # Steering control      
        delta = theta_e + theta_d

        return np.clip(delta, -self.max_steer, self.max_steer)

    def calc_egocar_yaw(self, currentPose):
        # compute Ego Car Yaw
        quaternion = (
            currentPose.orientation.x,
            currentPose.orientation.y,
            currentPose.orientation.z,
            currentPose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        return self.normalize_angle(yaw)

    def calc_path_yaw(self, Path, idx):
        # computes yaw of the path at index idx
        if idx >= len(Path.poses) - 1:
            return 0
        point_current = Path.poses[idx].pose.position
        point_next = Path.poses[idx+1].pose.position
        angle = math.atan2(point_next.y - point_current.y, point_next.x - point_current.x)
        return self.normalize_angle(angle)



    def calc_target_index(self, currentPath, currentPose):
        """
        Compute index in the trajectory list of the target.
        """

        if len(currentPath.poses) == 0:
            return 0, 0
        
        # Calc front axle position
        yaw = self.calc_egocar_yaw(currentPose)
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
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle



 # sudo docker run -it --network="host" -e CARLAVIZ_HOST_IP=localhost -e CARLA_SERVER_IP=localhost -e CARLA_SERVER_PORT=2000 mjxu96/carlaviz:latest
