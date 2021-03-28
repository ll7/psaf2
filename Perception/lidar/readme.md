## About
This node uses the Lidar-Sensor to compute to cars driving on the left and right lane. It  provides the information if there is the lane on the right and left is free to the [Behaviour-Node](https://github.com/ll7/psaf2/tree/main/Planning/behavior_agent) via a topic. In addition it provides a ROS-Service to compute whether a given lane (by id) is empty by using the lidar-sensor data.  

To compute the distance all points provided by the Lidar-Sensor are first transformed into map coordinates. Than they are filtered according to their distance to the respective lane. From the filtered Point-Cloud we check if there are any points left. If we still have some points, there is an object on the lane.
    
## Topics
Subscribes to
```
Topic                                           Message Type
/psaf/ego_vehicle/commonroad_map                     nav_msgs/Path.msg
/carla/ego_vehicle/odometry				nav_msgs/Odometry.msg
/carla/ego_vehicle/ldiar/lidar1/point_cloud     sensor_msgs/PointCloud2.msg 

```

Publishes to
```
Topic                                                   Message Type                    Description
/psaf/ego_vehicle/obstacle_on_left_lane                        std_msgs/Float64.msg            # distance to object on left lane (m)
/psaf/ego_vehicle/obstacle_on_right_lane                       std_msgs/Float64.msg            # distance to object on right lane (m)
/psaf/ego_vehicle/lidar/points                          sensor_msgs/PointCloud2.msg     # filtered pointcloud for debugging purposes
```
## Parameters
```python
...
        self.max_dist_lidar = 20 # maximum distance of lidar-sensor-data that should be considered
...
``` 

## How to launch this node?
```shell
roslaunch lidadr lidar.launch
```

## How to call the service from shell?
````shell
rosservice call check_lanelet_free "{keyword: value}"
e.g.: 
rosservice call check_lanelet_free "{isRoundabout: true, lanelet_id: 195}"
````

## How to call the service from python code?
```python
import from custom_carla_msgs.srv import TrafficOnLanelet  # import service message type
rospy.wait_for_service('check_lanelet_free')
check_lanelet_free = rospy.ServiceProxy("check_lanelet_free", TrafficOnLanelet)  # add service
check_lanelet_free(isRoundabout = True, lanelet_id = 195)  # call service with keyword argument
```

## Authors
Lukas Hartmann

Wolfgang Lang
