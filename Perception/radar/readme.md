## About
This node uses the Radar-Sensor to compute to the car driving in front. This information is then passed to the longitudinal controller inside the [steering_controllers-Package](https://github.com/ll7/psaf2/tree/main/Acting/steering_controllers) via the `/psaf/ego_vehicle/radar/distance`-Topic. It also provides the information if there is any car in front to the [Behaviour-Node](https://github.com/ll7/psaf2/tree/main/Planning/behavior_agent) via a topic. 

To compute the distance all points provided by the Radar-Sensor are first filtered according to their distance to the local-Path. We achieve decent Perception results in Turns and Intersection this way. From the filtered Point-Cloud we select whichever point is closest to the Ego-Vehicle (by Euclidean Distance) and return that distance.    
## Topics
Subscribes to
```
/psaf/ego_vehicle/localPath
/carla/ego_vehicle/radar/front/radar_points

```

Publishes to
```
/psaf/ego_vehicle/radar/distance # distance to object in front (m)
/psaf/ego_vehicle/radar/points # filtered pointcloud for debugging purposes
/psaf/ego_vehicle/bt/condition/slowed_by_car_in_front # True if there is an object on the path, False otherwise
```
## Parameters
```python
...
        self.max_dist_to_path = 2 # max distance a point can be from the path to be considered
        self.safety_time = 2.0 #time to wait if no obstacle detected
        self.safety_distance = 100 #distance to publish if no obstacle detected
...
``` 

## How to launch this node?
```shell
roslaunch radar radar.launch
```

## Authors
Wolfgang Lang


Valentin Höpfner