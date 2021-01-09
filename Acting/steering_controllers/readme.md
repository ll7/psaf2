## About
This package contains controller-code and a control loop, to compute and publish steering commands from a given path and target speed.
It uses a PID-Controller for longitudinal and a Stanley-Controller for lateral control.

## Topics
Subscribes to
```
/psaf/global_path 
/carla/ego_vehicle/target_speed
/carla/ego_vehicle/odometry
/carla/ego_vehicle/speedometer

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
At the moment there is no way to change parameters at runtime. 

## How to launch this node?
```shell
roslaunch steering_controllers steering_controllers.launch
```