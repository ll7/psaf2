# Praktikum zur Simulation von autonomen Fahrzeugen - Gruppe 2

The goal of this project was to develop an autonomous vehicle for the CARLA-simulator. The agent is to move collision-free and autonomously on various maps, on which other vehicles are also traveling, from an arbitrary starting point to an arbitrary destination. Two different modes are relevant for the evaluation: one mode in which traffic rules are observed, and one mode in which they are not. 

## Overview over the packages
We developed various ROS-nodes for the core-functionalities of our ego-vehicle. Each ROS-node is wrapped in a package, and those packages can be further grouped into one of three categories: Perception, Planning and Acting. An overview over the packages and some of their interfaces is provided in the following graphic.

![Overview over the packages](documentation/package_overview.svg)

Another (somewhat less readable) way to visualize the nodes and their communication - with each other and with the ros-bridge - is a rosgraph:

![Rosgraph]()


## Acting

### [ego_vehicle](Acting/ego_vehicle)
This node is used to launch the ros bridge and all the other packages. It also contains the config.json file, that describes the sensor-array.

### [steering_controllers](Acting/steering_controllers)
This node computes a [CarlaEgoVehicleControl-Message](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaEgoVehicleControl.msg) in each time step and publishes it to the ros-bridge. It uses a Stanley-Controller to follow the local path that is published by the local planner. The throttle command is computed through two PID-Controllers, one for distance and one for speed. This provides a primitive [ACC](https://en.wikipedia.org/wiki/Adaptive_cruise_control) that is also used to avoid collisions. 

## Planning

### global_planner

### local_planner

### behaviour_agent

## Perception



Packages werden mit "_" benannt.


# Installation

Julius bidde :)

## How to launch the ego vehicle?
```shell
roslaunch ego_vehicle ego_vehicle.launch
```
