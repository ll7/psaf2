import functools
import behavior_agent
import py_trees
from py_trees.behaviours import Running
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from behavior_agent import behaviours
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.decorators import Inverter

def grow_a_tree(role_name):

    root = Parallel("Root", children=[ 
                                        behaviours.topics2blackboard.create_node(role_name),
                                        Selector("Priorities", children=[
                                            Inverter(Selector("Avoid Collisions", children=[
                                                behaviours.avoid_collisions.NoObstacleAhead("No Obstacle Ahead?"),
                                                Selector("Collision Avoidance Action", children=[
                                                    behaviours.avoid_collisions.ReplanAroundObstacles("Replan around Obstacles"),
                                                    behaviours.avoid_collisions.EmergencyBrake("Emergency Brake")
                                                ])
                                            ])),
                                            Selector("Road Features", children=[
                                                Sequence("Intersection", children=[
                                                    behaviours.road_features.IntersectionAhead("Intersection Ahead"), 
                                                    Sequence("Intersection Actions", children=[
                                                        behaviours.intersection.Approach("Approach Intersection"),
                                                        behaviours.intersection.Wait("Wait Intersection"),
                                                        behaviours.intersection.Enter("Enter Intersection"),
                                                        behaviours.intersection.Leave("Leave Intersection")
                                                    ])
                                                ]),
                                                Sequence("Roundabout", children=[
                                                    behaviours.road_features.RoundaboutAhead("Roundabout Ahead"),
                                                    Sequence("Roundabout Actions", children=[
                                                        behaviours.roundabout.Approach("Approach Roundabout"),
                                                        behaviours.roundabout.Wait("Wait Roundabout"),
                                                        behaviours.roundabout.Enter("Enter Roundabout"),
                                                        behaviours.roundabout.Leave("Leave Roundabout")
                                                    ])
                                                ]),
                                                Sequence("Stop", children=[
                                                    behaviours.road_features.StopAhead("Stop Ahead"),
                                                    Sequence("Stop Actions", children=[
                                                        behaviours.stop.Approach("Approach Stop"),
                                                        behaviours.roundabout.Approach("Wait Stop"),
                                                        behaviours.roundabout.Approach("Leave Stop")
                                                    ])  
                                                ])

                                            ]),
                                            Selector("Laneswitching", children=[
                                                Inverter(Selector("Overtaking", children=[
                                                    py_trees.behaviours.Success("Not Slowed By Car in Front?"),
                                                    Selector("Number of Lanes", children=[
                                                        Sequence("Multi Lane", children=[
                                                            py_trees.behaviours.Failure("Multi Lane?")
                                                        ]),
                                                        Sequence("Single Lane", children=[
                                                            py_trees.behaviours.Failure("Single Lane with dotted Line?")
                                                        ])
                                                    ])
                                                ])),
                                                Sequence("Back to Right Lane", children=[
                                                    py_trees.behaviours.Failure("Right Lane Available")
                                                ])
                                            ]),

                                            py_trees.behaviours.Running(name="Idle")    
                                        ])
                                    ])
   
    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def main():
    """
    Entry point for the demo script.
    """
    rospy.init_node('behavior_tree', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    root = grow_a_tree(role_name)
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    
    if not behaviour_tree.setup(timeout=15):
        rospy.loginfo("Tree Setup failed")
        sys.exit(1)
    rospy.loginfo("tree setup worked")
    behaviour_tree.tick_tock(500)

if __name__ == "__main__":
    main()
