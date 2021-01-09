## About
This package launches carla-ros-bridge and spawns an ego vehicle.

## How do i add my package?
Just add your package to the corresponding section of launch/ego_vehicle.launch
For example:

```xml
...
  <!-- Add Launchfiles here -->
  
  <!-- Perception-->
  <include file="$(find lanelet-map-provider)/launch/lanelet-map-provider.launch">  </include>
  
  <!-- Planning -->
  <include file="$(find global-planner)/launch/global-planner.launch">  </include>  
  <include file="$(find global-planner)/launch/global-planner-target-publisher.launch">  </include>

  <!-- Acting-->
  <include file="$(find steering_controllers)/launch/vehicle_control.launch">  </include>
...
```


## How to launch this node?
```shell
roslaunch ego_vehicle ego_vehicle.launch
```
