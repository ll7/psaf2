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
Topic                                   Message Type
/psaf/ego_vehicle/localPath             nav_msgs/Path.msg
/psaf/ego_vehicle/radar/distance        std_msgs/Float64.msg
/psaf/ego_vehicle/target_speed          std_msgs/Float64.msg
/carla/ego_vehicle/odometry             nav_msgs/Odometry.msg

```

Publishes control commands to
```
Topic                                   Message Type
/carla/ego_vehicle/vehicle_control_cmd  carla_msgs/CarlaEgoVehicleControl.msg

```
## Controller Parameters
You can either pass custom controller-parameters to the VehicleController Object when initializing it, or change the standard parameters:

```python
...
        # speed controller parameters
        args_longitudinal = {'K_P': 0.25, 'K_D': 0.0, 'K_I': 0.1}
        # distance control parameters
        args_dist = {'K_P': 0.2, 'K_D': 0.0, 'K_I': 0.01}
        # Stanley control parameters
        args_lateral = {'k': 2.5, 'Kp': 1.0, 'L': 2.9, 'max_steer':30.0, 'min_speed':0.1}
...
``` 

## How to launch this node?
```shell
roslaunch steering_controllers steering_controllers.launch
```

## Authors
Valentin Höpfner
