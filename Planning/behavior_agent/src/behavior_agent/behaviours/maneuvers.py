import py_trees
import rospy

from custom_carla_msgs.srv import UpdateLocalPath

class SwitchLaneLeft(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SwitchLaneLeft, self).__init__(name)

    def setup(self, timeout):
        rospy.wait_for_service('update_local_path')
        self.update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        self.Successs = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.update_local_path(change_lane_left=True)


    def update(self):
        if self.Successs:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class SwitchLaneRight(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SwitchLaneRight, self).__init__(name)

    def setup(self, timeout):
        rospy.wait_for_service('update_local_path')
        self.update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        self.Successs = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.update_local_path(change_lane_right=True)


    def update(self):
        if self.Successs:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class Overtake(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Overtake, self).__init__(name)

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

class Cruise(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Cruise, self).__init__(name)

    def setup(self, timeout):
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


