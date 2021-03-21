# Praktikum zur Simulation von autonomen Fahrzeugen - Gruppe 2
Author: Valentin Höpfner

The goal of this project was to develop an autonomous vehicle for the CARLA-simulator. The agent is to move collision-free and autonomously on various maps, on which other vehicles are also traveling, from an arbitrary starting point to an arbitrary destination. Two different modes are relevant for the evaluation: one mode in which traffic rules are observed, and one mode in which they are not. 

## Overview over the packages
We developed various ROS-nodes for the core-functionalities of our ego-vehicle. Each ROS-node is wrapped in a package, and those packages can be further grouped into one of three categories: Perception, Planning and Acting. An overview over the packages and some of their interfaces is provided in the following graphic.

![Overview over the packages](documentation/package_overview.svg)

Another (somewhat less readable) way to visualize the nodes and their communication - with each other and with the ros-bridge - is a rosgraph:

![Rosgraph]()


## [Acting](Acting)

### [ego_vehicle](Acting/ego_vehicle)
This node is used to launch the ros bridge and all the other packages. It also contains the config.json file, that describes the sensor-array.

### [steering_controllers](Acting/steering_controllers)
This node computes a [CarlaEgoVehicleControl-Message](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaEgoVehicleControl.msg) in each time step and publishes it to the ros-bridge. It uses a Stanley-Controller to follow the local path that is published by the local planner. The throttle command is computed through two PID-Controllers, one for distance and one for speed. This provides a primitive [ACC](https://en.wikipedia.org/wiki/Adaptive_cruise_control) that is also used to avoid collisions. 

## [Planning](Planning)

### [global_planner](Planning/global_planner)
This node computes a global path from the position of the ego_vehicle to a given finish position. 

### [local_planner](Planning/local_planner)
This node provides a service, that can be called on to compute a local path. You can pass parameters to switch lanes or get into the right lane for turning on an intersection.

### [behaviour_agent](Planning/behaviour_agent)
This node contains the behaviour agent, that decides on the ego_vehicles actions (i.e. switching lane, overtaking) based on data from the Perception-nodes. It is based on a behaviour tree. It can influence the vehicles actions, by calling the local_planner to compute new paths, or passing a different target-speed to the steering_controllers node. 

## [Perception](Perception)

### [commonroad_map_provider](Perception/commonroad_map_provider)
This node parses the OpenDrive-File and provides it in the right format to the local- and global-planners.

### [radar](Perception/radar)
This node computes the distance to the car driving in front. Also considers the local path to filter the radar data. 

### [road_features](Perception/road_features)
This node computes informations about the current lanelet, like distance to the next intersection or the number of lanes. This information is provided to the behaviour agent. 



# Installation
Author: Julius Brandl

## How to launch the ego vehicle?
```shell
roslaunch ego_vehicle ego_vehicle.launch
```
Packages werden mit "_" benannt.