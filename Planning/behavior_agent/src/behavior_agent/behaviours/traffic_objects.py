import py_trees


class NotSlowedByCarInFront(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(NotSlowedByCarInFront, self).__init__(name)

    def setup(self, timeout):
        self.Successs = False
        return True



    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()


    def update(self):
        self.slowed = self.blackboard.get("/psaf/ego_vehicle/bt/condition/slowed_by_car_in_front")
        if self.slowed is None:
            return py_trees.common.Status.SUCCESS
        if self.slowed.data == True:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class WaitLeftLaneFree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(WaitLeftLaneFree, self).__init__(name)

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

class WaitRightLaneFree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(WaitRightLaneFree, self).__init__(name)

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

class NotSlowedByCarInFrontRight(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(NotSlowedByCarInFrontRight, self).__init__(name)

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

class OvertakingPossible(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(OvertakingPossible, self).__init__(name)

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


