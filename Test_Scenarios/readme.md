## About
This folder contains all Test_Scenarios as .sh-Files used during our project. Furthermore it contains different start- and goal-configurations as .yaml-files in the Folder "Yamls". 

The .sh-Files start:

1. Carla: 
	~/carla0.9.10.1/CarlaUE4.sh

2. ego_vehicle with special spawn_point (depends on scenario):
	roslaunch ego_vehicle ego_vehicle.launch spawn_point:='50.0, -7.45, 0.0, 0.0, 0.0, 180.0'"
	
3. Rviz (using -d 'your_config.rviz' you can start with your rviz-config): 
	rviz -d '/home/brandlju/Desktop/local_global_path.rviz'

4. local_path: 
	rosservice call update_local_path '{}'
	
5. rqt_console to debug:
	rqt_console

6. spawn npc (depends on scenario):
	python scenario_all_random.py -n 250
	
 
There are four different topics you can categorize these scenarios:

1. Roundabout:
	There is an scenario for each lane going into the roundabout in Town03.
	
2. Overtaking:
	It spawns only one other npc in front of our car to try to overtake it.
	
3. Distance/ACC:
	Similiar to Overtaking but here we wanted to check, if we can keep a given distance to the
	car in front.
	
4. Crossing:
	Spawns the car at a crossing.
	
## How to launch this node?
You only need to start the .sh-files by using this in /Test_Scenarios/:

```shell
./ego_vehicle_spawn_roundabout.sh
```
To run the .sh files, you may need to run this first:
sudo chmod u+x ego_vehicle_spawn_roundabout.sh 

## Authors
Wolfgang Lang
