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
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
        self.Successs = False
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        dist = self.blackboard.get("/psaf/ego_vehicle/distance_next_roundabout")
        # rospy.loginfo("we got a distance to next roundabout")
        if dist is not None:
            dist = dist.data
            if dist != np.inf:
                v = dist
                rospy.loginfo("changed target_speed for roundabout")
                self.target_speed_pub.publish(v)

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
        self.Successs = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Successs:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class Enter(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Enter, self).__init__(name)

    def setup(self, timeout):
        self.Successs = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Successs:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class Leave(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Leave, self).__init__(name)

    def setup(self, timeout):
        self.Successs = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Successs:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
