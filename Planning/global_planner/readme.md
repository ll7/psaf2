## About
Generates a global path from current position to a target position. If the ego vehicle is newly spawned the computation
starts immediately. To solve the problem the commonroad search package is used.

The node publishes the global path to /psag/ego_vehicle/global_path. It also publishes a GlobalPathLanelets message with
information about specific lanelets on the /psaf/ego_vehicle/global_path_lanelets topic. This message contains the lanelet ids
that are used by the ego vehicle if it follows the global path. Moreover, if a lanelet on the path has a adjacent those
lanelets are also included. All lanelets that are located in an intersection are published, too. 

## How to launch this node?
TODO: Subscriber für Lennart
```shell
roslaunch global-planner global-planner-example.launch
```

