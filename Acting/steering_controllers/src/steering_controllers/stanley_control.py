
import math
import numpy as np
from tf.transformations import euler_from_quaternion

class StanleyLateralController(object):  # pylint: disable=too-few-public-methods
    """
    StanleyLateralController implements longitudinal control using a PID.
    """

    def __init__(self):

        self.k = 0.5  # control gain
        self.Kp = 1.0  # speed proportional gain
        #self.dt = 0.1  # [s] time difference
        self.L = 2.9  # [m] Wheel base of vehicle
        self.max_steer = np.deg2rad(30.0)

    def run_step(self, currentPath, currentPose, dt):
        # compute Ego Car Position and Yaw
        # current_position = currentPose.position
        # quaternion = (
        #     currentPose.orientation.x,
        #     currentPose.orientation.y,
        #     currentPose.orientation.z,
        #     currentPose.orientation.w
        # )
        # _, _, yaw = euler_from_quaternion(quaternion)

        # idx = idxOfClosestWaypoint(currentPath, currentPose)

        # # position error
        # p_e = current_position - currentPath.poses[idx].pose.position

        # _, _, yaw = euler_from_quaternion(quaternion)
        return np.clip(1, -self.max_steer, self.max_steer)

    def idxOfClosestWaypoint(self, Path, Pose):
        #TODO
        return 0
