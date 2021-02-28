gnome-terminal --execute bash -c "~/carla0.9.10.1/CarlaUE4.sh -opengl; bash"
sleep 5s
gnome-terminal --execute bash -c "source ~/carla-ros-bridge/catkin_ws/devel/setup.bash; roslaunch ego_vehicle ego_vehicle.launch spawn_point:='239.3, 132.9, 0.0, 0.0, 0.0, 90.0'"
sleep 5s 
gnome-terminal --execute bash -c "rviz"
sleep 2s
gnome-terminal --execute bash -c "python scenario_all_random.py -n 200"
sleep 1s
gnome-terminal --execute bash -c "rostopic pub /carla/ego_vehicle/target_speed std_msgs/Float64 90.0"




