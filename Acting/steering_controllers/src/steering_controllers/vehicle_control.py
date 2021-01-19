from collections import deque
import math
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Pose
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Float32

from steering_controllers.pid_control import PIDLongitudinalController 
from steering_controllers.stanley_control import StanleyLateralController




class VehicleController(object):  # pylint: disable=too-few-public-methods
    """
    VehicleController is the combination of two controllers (lateral and longitudinal)
    to perform the low level control a vehicle from client side
    """

    def __init__(self, role_name, target_speed, args_longitudinal=None, args_lateral=None):

        self._current_speed = 0.0  # Km/h
        self._current_pose = Pose()
        self._route = Path()
        self._target_speed = target_speed

        if not args_longitudinal:
            args_longitudinal = {'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
        if not args_lateral:
            args_lateral = {'k': 0.5, 'Kp': 1.0, 'L': 2.9, 'max_steer':30.0}

        self._lon_controller = PIDLongitudinalController(**args_longitudinal)
        self._lat_controller = StanleyLateralController(**args_lateral)
        self._last_control_time = rospy.get_time()

        
        self._route_subscriber = rospy.Subscriber(
            f"/psaf/{role_name}/global_path", Path, self.path_updated)

        self._target_speed_subscriber = rospy.Subscriber(
            "/carla/{}/target_speed".format(role_name), Float64, self.target_speed_updated)
        
        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(role_name), Odometry, self.odometry_updated)

        self._odometry_subscriber = rospy.Subscriber(
            "/carla/{}/speedometer".format(role_name), Float32, self.speed_updated)

        self.vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd".format(role_name), CarlaEgoVehicleControl, queue_size=1)
        
    def run_step(self, target_speed, current_speed):
        """
        Execute one step of control invoking longitudinal
        PID controller to reach a given target_speed.
        :param target_speed: desired vehicle speed
        """
        current_time = rospy.get_time()
        dt = current_time-self._last_control_time
        if dt == 0.0:
            dt = 0.000001
        control = CarlaEgoVehicleControl()
        throttle = self._lon_controller.run_step(self._target_speed, self._current_speed, dt)
        steering = self._lat_controller.run_step(self._route, self._current_pose, self._current_speed)
        self._last_control_time = current_time
        control.throttle = throttle
        control.steer = steering
        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


    def odometry_updated(self, odo):
        """
        callback on new odometry data
        """

        self._current_pose = odo.pose.pose
    
    def speed_updated(self, speed):
        """
        callback on new spped
        converts from m/s to km/h
        """
        self._current_speed = speed.data*3.6

    def target_speed_updated(self, target_speed):
        """
        callback on new target speed
        """
        rospy.loginfo("New target speed received: {}".format(target_speed.data))
        self._target_speed = target_speed.data

    def path_updated(self, path):
        """
        callback on new route
        """
        rospy.loginfo("New plan with {} waypoints received.".format(len(path.poses)))
        self._route = path


    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            control = self.run_step(self._target_speed, self._current_speed)
            if control:
                control.steer = -control.steer
                self.vehicle_control_publisher.publish(control)
            else:
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass

def main():
    rospy.init_node('vehicle_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    target_speed = rospy.get_param("~target_speed", 0)
    controller = VehicleController(role_name, target_speed)
    try:
        controller.run()
    finally:
        del controller
    rospy.loginfo("Done")

if __name__ == "__main__":
    main()


