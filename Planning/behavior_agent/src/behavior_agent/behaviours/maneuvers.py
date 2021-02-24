import py_trees


class SwitchLaneLeft(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SwitchLaneLeft, self).__init__(name)

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

class SwitchLaneRight(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SwitchLaneRight, self).__init__(name)

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


