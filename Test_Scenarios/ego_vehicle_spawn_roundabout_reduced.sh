gnome-terminal --execute bash -c "~/carla0.9.10.1/CarlaUE4.sh -opengl; bash"
sleep 5s
gnome-terminal --execute bash -c "source ~/carla-ros-bridge/catkin_ws/devel/setup.bash; roslaunch ego_vehicle ego_vehicle.launch spawn_point:='5.0, 80.7, 0.0, 0.0, 0.0, 90.0'"
gnome-terminal --execute bash -c "rviz"
sleep 2s





