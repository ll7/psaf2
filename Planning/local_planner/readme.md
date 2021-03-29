## Local Planner
ROS-Service to update the local path that the ego vehicle follows. 

The local path is updated with help of the previously generated global path and the commonroad map.
To do so, the node receives the current position of the ego vehicle and computes the corresponding lanelet on the commonroad
map. With this and the information about the global path a local path is created. The local path includes a route from
the current position until the end of the next lanelet. 

By using a local planner instead of a global one has some advantages. E.g. the ego vehicle is able to stay on an adjacent lane next to the lane 
given by the global path for easier overtaking. The local planner is also responsible to switch to the correct lane if
necessary before entering an intersection. The local planner is responsible to plan lane changes, intersection and roundabout behaviour but
it only takes static obstacles into account. The goal is to follow the main path while using the best possible lane.

Planning for the roundabout works as follows: Before the roundabout a switch to the right-most lane is done (if 
necceesary) and the distance to the outer circle is computed so car can stop at roundabout entry. From there on
the path on the outer ring through the roundabout is planned.

The generated local path is then published to /psaf/ego_vehicle/localPath.

## Topics
Subscribes to
```
Topic                                       Message Type
/carla/ego_vehicle/odometry                 nav_msgs/Odometry.msg
/psaf/ego_vehicle/global_path_lanelets      custom_carla_msgs/GlobalPathLanelets.msg
/psaf/ego_vehicle/commonroad_map            std_msgs/String.msg
```

Publishes to
```
Topic                                       Message Type
/psaf/ego_vehicle/local_path              nav_msgs/Path.msg
/psaf/ego_vehicle/first_lanelet_roundabout  std_msgs/Int32.msg
/psaf/ego_vehicle/distance_next_roundabout  custom_carla_msgs/NextLanelet.msg
/psaf/ego_vehicle/distance_exit_roundabout  geometry_msgs/Point.msg
```

## How to call the service from shell?
````shell
rosservice call update_local_path "{keyword: value}"
e.g.: 
rosservice call update_local_path "{change_lane_right: true}"
````

## How to call the service from python code?
```python
import from custom_carla_msgs.srv import UpdateLocalPath  # import service message type
rospy.wait_for_service('update_local_path')
update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)  # add service
update_local_path(change_lane_right=True)  # call service with keyword argument
```


