#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros

from std_msgs.msg import Float64, String, Bool
from nav_msgs.msg import Odometry

from custom_carla_msgs.msg import GlobalPathLanelets, LaneStatus



def create_node(role_name):
    topics =[
            {'name':f"/carla/{role_name}/odometry", 'msg':Odometry, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
            {'name':f"/carla/{role_name}/target_speed", 'msg':Float64, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER}, 
            {'name':f"/psaf/{role_name}/obstacle", 'msg':String, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
            {'name':f"/psaf/{role_name}/bt/condition/slowed_by_car_in_front", 'msg':Bool, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER} ,
            {'name':f"/psaf/{role_name}/distance_next_intersection", 'msg':Float64, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
            {'name':f"/psaf/{role_name}/global_path_lanelets", 'msg':GlobalPathLanelets ,'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
            {'name':f"/psaf/{role_name}/lane_status", 'msg':GlobalPathLanelets ,'clearing-policy': py_trees.common.ClearingPolicy.NEVER}
         ]
    topics2blackboard = py_trees.composites.Parallel("Topics to Blackboard")
    for topic in topics:
        topics2blackboard.add_child(py_trees_ros.subscribers.ToBlackboard(  name=topic['name'],
                                                                            topic_name=topic['name'], 
                                                                            topic_type=topic['msg'], 
                                                                            blackboard_variables={topic['name']: None}, 
                                                                            clearing_policy=topic['clearing-policy']))
    return topics2blackboard