## About
Service to generate a global path from current position to a target position. If the ego vehicle is newly spawned the computation
starts immediately. To solve the problem the commonroad search package is used.

The node publishes the global path to /psag/ego_vehicle/global_path. It also publishes a GlobalPathLanelets message with
information about specific lanelets on the /psaf/ego_vehicle/global_path_lanelets topic. This message contains the lanelet ids
that are used by the ego vehicle if it follows the global path. Moreover, if a lanelet on the path has a adjacent those
lanelets are also included. All lanelets that are located in an intersection are published, too. 

## How to call the service from shell?
````shell
rosservice call update_global_path "{}"
````

## How to call the service from python code?
```python
import from custom_carla_msgs.srv import UpdateGlobalPath  # import service message type
rospy.wait_for_service('update_global_path')
update_global_path = rospy.ServiceProxy("update_local_path", UpdateGlobalPath)  # add service
update_global_path()  # call service with keyword argument
```


