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
        self.traffic_light_detected = False
        self.traffic_light_distance = np.inf
        self.traffic_light_status = ''
        self.virtual_stopline_distance = np.inf
        self.target_speed_pub.publish(30.0)
        self.last_virtual_distance = np.inf
        rospy.loginfo("start approaching behavior")

    def update(self):
        # Update Light Info
        light_status_msg = self.blackboard.get("/psaf/ego_vehicle/traffic_light")
        if light_status_msg is not None:
            self.traffic_light_status = light_status_msg.color
            rospy.loginfo(f"Light Status: {self.traffic_light_status}")
            self.traffic_light_distance = light_status_msg.distance
            rospy.loginfo(f"Light distance: {self.traffic_light_distance}")
        # Update stopline Info
        _dis = self.blackboard.get("/psaf/ego_vehicle/stopline_distance")
        if _dis is not None:
            self.stopline_distance = _dis.data
            rospy.loginfo(f"Stopline distance: {self.stopline_distance}")
        
        # calculate virtual stopline
        if self.stopline_distance != np.inf:
            self.virtual_stopline_distance = self.stopline_distance
        else:
            self.virtual_stopline_distance = self.traffic_light_distance
        
        # calculate speed needed for stopping
        v_stop = max(5., (self.virtual_stopline_distance/30)**1.5 * 50)
        if v_stop > 30:
            v_stop = 30 
        if self.virtual_stopline_distance < 3.5:
            v_stop = 0
        # stop when there is no or red/yellow traffic light
        if self.traffic_light_status == '' or self.traffic_light_status == 'red' or self.traffic_light_status == 'yellow':
            rospy.loginfo(f"slowing down: {v_stop}")
            self.target_speed_pub.publish(v_stop)
        
        # approach slowly when traffic light is green
        if self.traffic_light_status == 'green':
            self.target_speed_pub.publish(30)
        
        # get speed
        odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        speed = np.sqrt(odo.twist.twist.linear.x ** 2 + odo.twist.twist.linear.y ** 2 + odo.twist.twist.linear.z ** 2)*3.6


        if self.virtual_stopline_distance > 5:
            # too far
            return py_trees.common.Status.RUNNING
        elif speed < 2 and self.virtual_stopline_distance < 5:
            # stopped
            return py_trees.common.Status.SUCCESS
        elif speed > 5 and self.virtual_stopline_distance < 3.5:
            # running over line
            return py_trees.common.Status.SUCCESS
        elif self.last_virtual_distance == self.virtual_stopline_distance and self.virtual_stopline_distance < 10:
            # ran over line
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        

        if (self.last_virtual_distance == self.virtual_stopline_distance and self.virtual_stopline_distance < 10) or self.virtual_stopline_distance < 3.5:
            return py_trees.common.Status.SUCCESS
        elif (self.virtual_stopline_distance == np.inf and (rospy.get_time() - self.start_time) > 15) or (rospy.get_time() - self.start_time) > 120:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING




        return py_trees.common.Status.FAILURE
        light_status_msg = self.blackboard.get("/psaf/ego_vehicle/traffic_light")
        if light_status_msg is None:
            return py_trees.common.Status.SUCCESS
        else:
            light_status = light_status_msg.color
            rospy.loginfo(f"Light Status: {light_status}")
            light_distance = light_status_msg.distance
            rospy.loginfo(f"Light distance: {light_distance}")

        # if no stop line seen within 3 seconds, return success
        if not self.stopline_detected and (rospy.get_time() - self.start_time) > 15:
            rospy.loginfo("time up for waiting for stop line")
            return py_trees.common.Status.SUCCESS

        # check for stopline update
        _dis = self.blackboard.get("/psaf/ego_vehicle/stopline_distance")

        if _dis is not None:
            self.stopline_distance = _dis.data
            rospy.loginfo(f"Stopline distance: {self.stopline_distance}")

        
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
            if light_status == "red" or light_status == "yellow" or light_status == "":
                self.target_speed_pub.publish(0)

        # calculate speed depending on the distance to stop line
        if self.stopline_detected and self.stopline_distance != np.inf or light_status == "":
            v = 30 * self.stopline_distance
            if light_status == "red" or light_status == "yellow":
                self.target_speed_pub.publish(v)

        # check if speed is 0
        self.odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        self.speed = np.sqrt(
            self.odo.twist.twist.linear.x ** 2 + self.odo.twist.twist.linear.y ** 2 + self.odo.twist.twist.linear.z ** 2)*3.6
        if self.speed < 2:
            return py_trees.common.Status.SUCCESS
        elif self.stopline_detected and self.stopline_distance == np.inf:
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
        return True

    def update(self):
        light_status = self.blackboard.get("/psaf/ego_vehicle/traffic_light")
        if light_status is None:
            return py_trees.common.Status.SUCCESS
        else:
            light_status = light_status.color
        if light_status == "red" or light_status == "yellow":
            self.target_speed_pub.publish(0)
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
        
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
