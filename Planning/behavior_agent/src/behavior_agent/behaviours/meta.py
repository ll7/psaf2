#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import py_trees
import numpy as np
from std_msgs.msg import Float64
from custom_carla_msgs.srv import UpdateGlobalPath, UpdateLocalPath


class Start(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Start, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        rospy.wait_for_service('update_global_path')
        self.update_global_path = rospy.ServiceProxy("update_global_path", UpdateGlobalPath)
        rospy.wait_for_service('update_local_path')
        self.update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
        return True

    def initialise(self):
        return True

    def update(self):
        success_global_path = self.update_global_path()
        rospy.loginfo(success_global_path)
        if success_global_path:
            bb_dist = self.blackboard.get("/psaf/ego_vehicle/distance_next_intersection")
            if bb_dist is not None:
                dist = bb_dist.data
                if dist == np.inf:
                    success_local_path = self.update_local_path(leave_intersection=True)
                else:
                    success_local_path = self.update_local_path(approach_intersection=True)
                if success_local_path:
                    rospy.loginfo("Everything is fine. We can start now!")
                    self.target_speed_pub.publish(50.0)
                    return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class End(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(End, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
        return True

    def initialise(self):
        self.target_speed_pub.publish(0.0)

    def update(self):
        odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        if odo is None:
            return py_trees.common.Status.FAILURE
        current_pos = np.array([odo.pose.pose.position.x, odo.pose.pose.position.y])
        target_pos = np.array([rospy.get_param('/competition/goal/position/x', 10),
                               rospy.get_param('/competition/goal/position/y', 50)])
        dist = np.linalg.norm(current_pos - target_pos)
        if dist < 5:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class Rules(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Rules, self).__init__(name)

    def setup(self, timeout):
        return True

    def initialise(self):
        return True

    def update(self):
        rules = rospy.get_param('/competition/traffic_rules', True)
        if rules:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class RespawnOrFinish(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(RespawnOrFinish, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.last_init_pose = None
        return True

    def initialise(self):
        return True

    def update(self):
        """
        Checks if car was respawned or car reached target.
        :return:
        """
        init_pose = self.blackboard.get("/initialpose")

        if init_pose is not None:
            if self.last_init_pose is not None and init_pose != self.last_init_pose:
                self.target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
                self.target_speed_pub.publish(0.0)
                rospy.loginfo(f"New spawn at {init_pose.pose.pose}")
                self.last_init_pose = init_pose
                return py_trees.common.Status.SUCCESS
            else:
                self.last_init_pose = init_pose

        odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        if odo is None:
            return py_trees.common.Status.FAILURE
        current_pos = np.array([odo.pose.pose.position.x, odo.pose.pose.position.y])
        target_pos = np.array([rospy.get_param('/competition/goal/position/x', 10),
                               rospy.get_param('/competition/goal/position/y', 50)])
        dist = np.linalg.norm(current_pos - target_pos)
        if dist < 5:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))