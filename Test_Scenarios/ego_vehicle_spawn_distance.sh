gnome-terminal --execute bash -c "~/carla0.9.10.1/CarlaUE4.sh -opengl; bash"
sleep 5s
gnome-terminal --execute bash -c "source ~/carla-ros-bridge/catkin_ws/devel/setup.bash; roslaunch ego_vehicle ego_vehicle.launch spawn_point:='-20.0, 207.0, 0.0, 0.0, 0.0, 0.0'; bash"
sleep 5s 
gnome-terminal --execute bash -c "rviz; bash"
sleep 2s
gnome-terminal --execute bash -c "python scenario_overtaking.py; bash"
sleep 1s
gnome-terminal --execute bash -c "rostopic pub /psaf/ego_vehicle/target_speed std_msgs/Float64 30.0"




