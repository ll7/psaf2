#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry



class IntersectionAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(IntersectionAhead, self).__init__(name)

    def setup(self, timeout):
        self.currentstart = 0.0
        self.blackboard = py_trees.blackboard.Blackboard()
        self.lastdist = 0.0
        return True

    def initialise(self):
        self.dist = 0


    def update(self):
        # TODO Write data field to blackboard directly
        bb = self.blackboard.get("/psaf/ego_vehicle/distance_next_intersection")
        if bb is None:
            return py_trees.common.Status.FAILURE
        else:
            self.dist = bb.data
        if self.dist < 20:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
            

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class RoundaboutAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(RoundaboutAhead, self).__init__(name)

    def setup(self, timeout):
        self.Roundabout = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Roundabout:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class StopAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(StopAhead, self).__init__(name)

    def setup(self, timeout):
        self.Stop = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Stop:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class MultiLane(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MultiLane, self).__init__(name)

    def setup(self, timeout):
        self.Success = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class SingleLineDotted(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SingleLineDotted, self).__init__(name)

    def setup(self, timeout):
        self.Success = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class RightLaneAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(RightLaneAvailable, self).__init__(name)

    def setup(self, timeout):
        self.Success = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class LeftLaneAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LeftLaneAvailable, self).__init__(name)

    def setup(self, timeout):
        self.Success = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        if self.Success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))