import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from behavior_agent.agent_behaviours import Foo, Testbev
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

def create_my_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.
    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Parallel("Tutorial")
    topics2blackboard = py_trees.composites.Parallel("Topics to Blackboard")
    avoid_collisions = py_trees.composites.Selector("Avoid Collisions")
    moving = Foo("Moving")
    priorities = py_trees.composites.Selector("Priorities")
    idle = py_trees.behaviours.Running(name="Idle")
    movingfast = Testbev("movingfast")
    topics = [{'name':"/carla/ego_vehicle/odometry", 'msg':Odometry, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
              {'name':"/carla/ego_vehicle/target_speed", 'msg':Float64, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER} 
                ]
    for topic in topics:
        topics2blackboard.add_child(py_trees_ros.subscribers.ToBlackboard(topic['name'],topic['name'], topic['msg'], {topic['name']: None}, clearing_policy=topic['clearing-policy']))
    root.add_child(avoid_collisions)
    root.add_child(topics2blackboard)
    avoid_collisions.add_child(moving)
    avoid_collisions.add_child(movingfast)
    root.add_child(priorities)
    priorities.add_child(idle)
    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def main():
    """
    Entry point for the demo script.
    """
    rospy.init_node('behavior_tree', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    root = create_my_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    
    if not behaviour_tree.setup(timeout=15):
        rospy.loginfo("Tree Setup failed")
        sys.exit(1)
    rospy.loginfo("tree setup worked")
    behaviour_tree.tick_tock(500)

if __name__ == "__main__":
    main()
