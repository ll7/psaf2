## About
This package contains controller-code and a control loop, to compute and publish steering commands from a given path and target speed.
It uses a Stanley-Controller for lateral control. Its working principle is described in [the Wiki](https://github.com/ll7/psaf2/wiki/Path-Tracking-Algorithmen).

For longitudinal control a combination of two PID-Controllers(distance and speed) is used. The control loop is visualized in the following image:
![](https://github.com/ll7/psaf2/blob/main/documentation/steering_controllers/longitudinal_control.svg)
In conjunction with distance data from the [Radar-Node](https://github.com/ll7/psaf2/tree/main/Perception/radar) this yields a simple but effective implementation of ACC (Adaptive Cruise Control), that helps with adjusting to traffic and avoids collisions. 

The controller code is stored in seperate Files for easy reusability.

## Topics
Subscribes to
```
/psaf/ego_vehicle/localPath
/psaf/ego_vehicle/radar/distance
/carla/ego_vehicle/target_speed
/carla/ego_vehicle/odometry

```

Publishes control commands to
```
/carla/{}/vehicle_control_cmd

```
## Controller Parameters
You can either pass custom controller-parameters to the VehicleController Object when initializing it, or change the standard parameters:

```python
...
    if not args_longitudinal:
        args_longitudinal = {'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
    if not args_lateral:
        args_lateral = {'k': 0.5, 'Kp': 1.0, 'L': 2.9, 'max_steer':30.0}
...
``` 

## How to launch this node?
```shell
roslaunch steering_controllers steering_controllers.launch
```

## Authors
@halentin