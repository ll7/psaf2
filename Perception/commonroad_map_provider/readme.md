## About

Reads opendrive map string from /carla/WorldInfo, converts it to a commonorad map and saves it as a .xml file.
The path to this converted map is then published. The default path to the file is ~/.ros/commonroad_map.xml.

## Topics
Subscribes to
```
Topic                                   Message Type
/carla/world_info                       carla_msgs/CarlaWorldInfo.msg
```

Publishes to
```
Topic                                   Message Type
/psaf/ego_vehicle/commonroad_map        std_msgs/String.msg
```

## How to launch this node?
If you just want to create a commonroad map of the current opendrive map you can use:
```shell
roslaunch commonroad_map_provider commonroad_map_provider_example.launch
```


## Authors
Julius Brandl

Lukas Hartmann
