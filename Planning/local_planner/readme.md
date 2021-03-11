## Local Planner
ROS-Service to update the local path the ego vehicle follows. 

The local path is updated with help of the previously generated global path and the commonroad map.
To do so the node receives the current position of the ego vehicle and computes the corresponding lanelet on the commonroad
map. With this and the information about the global path a local path is created. The local path includes a route from
the current position until the end of the next lanelet.

Using a local planner instead of a global one has some advantages. E.g. the ego vehicle is able to stay on a adjacent lane next to the lane 
given by the global path for easier overtaking. The local planner is also responsible to switch to the correct lane if
necessary before entering an intersection.  

The generated local path is then published to /psaf/ego_vehicle/localPath.

## How to call the service from shell?
````shell
rosservice call /update_local_path "{keyword = value}"
e.g.: 
rosservice call /update_local_path "{change_lane_right = true}"
````

## How to call the service from python code?
```python
import from custom_carla_msgs.srv import UpdateLocalPath  # import service message type
rospy.wait_for_service('update_local_path')
update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)  # add service
update_local_path(change_lane_right=True)  # call service with keyword argument
```


