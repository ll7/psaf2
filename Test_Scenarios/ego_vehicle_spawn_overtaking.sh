gnome-terminal --execute bash -c "~/carla0.9.10.1/CarlaUE4.sh -opengl; bash"
sleep 5s
gnome-terminal --execute bash -c "source ~/carla-ros-bridge/catkin_ws/devel/setup.bash; roslaunch ego_vehicle ego_vehicle.launch spawn_point:='10.0, 207.0, 0.0, 0.0, 0.0, 0.0'"
sleep 3s 
gnome-terminal --execute bash -c "rviz -d '/home/brandlju/Desktop/local_global_path.rviz'"
sleep 2s
gnome-terminal --execute bash -c "python scenario_overtaking.py"
sleep 1s
gnome-terminal --execute bash -c "rqt_console"
sleep 5s
gnome-terminal --execute bash -c "rosscervice call update_local_path '{}'"
