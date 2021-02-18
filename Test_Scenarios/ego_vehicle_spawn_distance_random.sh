gnome-terminal --execute bash -c "~/carla0.9.10.1/CarlaUE4.sh -opengl; bash"
sleep 5s
gnome-terminal --execute bash -c "source ~/carla-ros-bridge/catkin_ws/devel/setup.bash; roslaunch ego_vehicle ego_vehicle.launch spawn_point:='-20.0, 207.5, 0.0, 0.0, 0.0, 0.0'"
sleep 5s 
gnome-terminal --execute bash -c "rviz"
sleep 2s
<<<<<<< HEAD
<<<<<<< HEAD
gnome-terminal --execute bash -c "python scenario_all_random.py -n 200"
sleep 1s
gnome-terminal --execute bash -c "rostopic pub /carla/ego_vehicle/target_speed std_msgs/Float64 60.0"
=======
gnome-terminal --execute bash -c "python scenario_all_random.py -n 20"
=======
gnome-terminal --execute bash -c "python scenario_all_random.py -n 200"
>>>>>>> filter pointcloud
sleep 1s
gnome-terminal --execute bash -c "rostopic pub /carla/ego_vehicle/target_speed std_msgs/Float64 50.0"
>>>>>>> combined distance and speed_limit




