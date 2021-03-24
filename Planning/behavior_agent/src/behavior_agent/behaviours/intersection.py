import py_trees
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from custom_carla_msgs.srv import UpdateLocalPath

import rospy


class Approach(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Approach, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/target_speed", Float64, queue_size=1)
        rospy.wait_for_service('update_local_path')
        self.update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        rospy.loginfo("Approaching Intersection")
        self.update_local_path(approach_intersection=True)
        self.start_time = rospy.get_time()
        self.stopline_detected = False
        self.stopline_distance = np.inf
        self.target_speed_pub.publish(30.0)
        rospy.loginfo("start approaching behavior")

    def update(self):
        # if no stop line seen within 3 seconds, return success
        if not self.stopline_detected and (rospy.get_time() - self.start_time) > 15:
            rospy.loginfo("time up for waiting for stop line")
            return py_trees.common.Status.SUCCESS

        # check for stopline update
        _dis = self.blackboard.get("/psaf/ego_vehicle/stopline_distance")
        if _dis is not None:
            self.stopline_distance = _dis.data

        # check if stop line detected
        if self.stopline_detected is False and self.stopline_distance != np.inf:
            self.stopline_detected = True
            rospy.loginfo("stopline detected")

        # if no stop line, wait for one to appear or time to run out
        if not self.stopline_detected:
            rospy.loginfo("waiting for stop line or time out")
            return py_trees.common.Status.RUNNING

        # if stop line has been detected and now there is none, we've passed it and need to break immediately
        if self.stopline_detected and self.stopline_distance == np.inf:
            rospy.loginfo("ran over stop line")
            self.target_speed_pub.publish(0)

        # calculate speed depending on the distance to stop line
        if self.stopline_detected and self.stopline_distance != np.inf:
            v = 30 * self.stopline_distance
            self.target_speed_pub.publish(v)

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
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/target_speed", Float64, queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        self.target_speed_pub.publish(0)

    def update(self):
        self.odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        self.speed =  np.sqrt(
            self.odo.twist.twist.linear.x ** 2 + self.odo.twist.twist.linear.y ** 2 + self.odo.twist.twist.linear.z ** 2)*3.6
        if self.speed < 5:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class Enter(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Enter, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/target_speed", Float64, queue_size=1)
        rospy.wait_for_service('update_local_path')
        self.update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        self.target_speed_pub.publish(50.0)

    def update(self):
        odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        speed = np.sqrt(
            odo.twist.twist.linear.x ** 2 + odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2)*3.6
        if speed > 10:
            self.update_local_path(leave_intersection=True)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class Leave(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Leave, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        rospy.wait_for_service('update_local_path')
        self.update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        return True

    def initialise(self):
        return True

    def update(self):
        next_lanelet_msg = self.blackboard.get("/psaf/ego_vehicle/next_lanelet")
        if next_lanelet_msg is None:
            return py_trees.common.Status.FAILURE
        if next_lanelet_msg.distance < 15 and not next_lanelet_msg.isInIntersection:
            rospy.loginfo("Leave leave behaviour!")
            self.update_local_path(leave_intersection=True)
            return py_trees.common.Status.FAILURE
        else:
            rospy.loginfo("Stay in leave behaviour!")
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
