gnome-terminal --execute bash -c "~/carla0.9.10.1/CarlaUE4.sh -opengl -quality-level=Low; bash"
sleep 5s
gnome-terminal --execute bash -c "roslaunch steering_controllers steering_controllers_example.launch; bash"
gnome-terminal --execute bash -c "docker run -it -e CARLAVIZ_HOST_IP=localhost -e CARLA_SERVER_IP=host.docker.internal -e CARLA_SERVER_PORT=2000 -p 8080-8081:8080-8081 -p 8089:8089 mjxu96/carlaviz:latest; bash"
gnome-terminal --execute bash -c "rviz; bash"
gnome-terminal --execute bash -c "firefox 127.0.0.1:8080"


