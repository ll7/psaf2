import numpy as np
import math
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path

def calc_path_yaw(Path, idx):
        # computes yaw of a path at index idx
        if idx >= len(Path.poses) - 1:
            #this should probably throw an exception
            return 0

        point_current = Path.poses[idx].pose.position
        point_next = Path.poses[idx+1].pose.position
        angle = math.atan2(point_next.y - point_current.y, point_next.x - point_current.x)
        return normalize_angle(angle)

def calc_egocar_yaw(currentPose):
    # compute Ego Car Yaw
    quaternion = (
        currentPose.orientation.x,
        currentPose.orientation.y,
        currentPose.orientation.z,
        currentPose.orientation.w
            )
    _, _, yaw = euler_from_quaternion(quaternion)
    return normalize_angle(yaw)

def normalize_angle(angle):
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
