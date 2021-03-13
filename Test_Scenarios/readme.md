to run Test_Scenarios:

Crossing: 	
1. run: ./ego_vehicle_spawn_crossing.sh
2. run: python3 scenario_crossing.py

Overtaking:
1. run: ./ego_vehicle_spawn_overtaking.sh
2. run: python3 scenario_overtaking.py

Same cars on same positions:
1. run: roslaunch ego_vehicle ego_vehicle.launch
2. run: python3 scenario_same_cars_on_same_positions.py -n 200

All random:
1. run: roslaunch ego_vehicle ego_vehicle.launch
2. run: python3 scenario_all_random.py -n 200

To run the .sh files, you need to run:
sudo chmod u+x ego_vehicle_spawn_crossing.sh 
sudo chmod u+x ego_vehicle_spawn_overtaking.sh


