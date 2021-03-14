import py_trees
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import rospy
from datetime import datetime


class Approach(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Approach, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.start_time = datetime.now()
        self.stopline_detected = False
        self.trafficlight_detected = False
        self.stopline_distance = np.inf
        self.trafficlight_distance = np.inf
        rospy.loginfo("start approaching behavior")

    def update(self):
        # if no stop line seen within 3 seconds, return success
        if not self.stopline_detected and not self.trafficlight_detected and (datetime.now() - self.start_time).total_seconds() > 15:
            rospy.loginfo("time up for waiting for stop line or traffic light")
            return py_trees.common.Status.SUCCESS
        
        # check for stopline update
        _dis = self.blackboard.get("/psaf/ego_vehicle/stopline_distance")
        if _dis is not None:
            self.stopline_distance = _dis.data

        # check for stop line at traffic lights update
        _dis_tl = 50  # self.blackboard.get("/psaf/ego_vehicle/??????")
        if _dis_tf is not None:
            if stopline_distance != np.inf:
                self.trafficlight_distance = _dis.data
            else:
                self.trafficlight_distance = _dis_tl

        # check if stop line detected    
        if self.stopline_detected is False and self.stopline_distance != np.inf:
            self.stopline_detected = True
            rospy.loginfo("stopline detected")

        # check if traffic light detected
        if self.trafficlight_detected is False and self.trafficlight_distance != np.inf:
            self.trafficlight_detected = True
            rospy.loginfo("traffic_light detected")

        # if no stop line or no traffic light, wait for one to appear or time to run out
        if not self.stopline_detected or not self.trafficlight_detected:
            rospy.loginfo("waiting for stop line or for traffic light or time out")
            return py_trees.common.Status.RUNNING

        # if stop line has been detected and now there is none, we've passed it and need to break immediately
        if (self.stopline_detected and self.stopline_distance == np.inf) \
                or (self.trafficlight_detected and self.trafficlight_distance == np.inf):
            rospy.loginfo("ran over stop line")
            self.target_speed_pub.publish(0)

        # calculate speed depending on the distance to stop line
        if (self.stopline_detected and self.stopline_distance != np.inf) \
                or (self.trafficlight_detected and self.trafficlight_distance != np.inf):
            v = 30 * (self.stopline_distance ** 2)
            self.target_speed_pub.publish(v)
            rospy.loginfo(f"slowed down to {v}")

        # check if speed is 0
        self.odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        self.speed =  np.sqrt(
            self.odo.twist.twist.linear.x ** 2 + self.odo.twist.twist.linear.y ** 2 + self.odo.twist.twist.linear.z ** 2)*3.6
        if self.speed < 1:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Wait, self).__init__(name)

    def setup(self, timeout):
        return True

    def initialise(self):

        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        #check if the traffic light is not green
        self.tl = self.blackboard.get("/psaf/ego_vehicle/perception_info")
        if ('green' not in self.tl.values):
            return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class Enter(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Enter, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
        self.target_speed_pub.publish(30.0)

    def update(self):
        odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        speed =  np.sqrt(
            odo.twist.twist.linear.x ** 2 + odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2)*3.6
        if speed > 10:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class Leave(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Leave, self).__init__(name)

    def setup(self, timeout):
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        return py_trees.common.Status.SUCCESS
           # return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
