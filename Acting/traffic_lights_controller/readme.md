Reads traffic light status (erkennt eine rote Ampel, die sich im Begrenzungsrahmen des Autos befindet)
Publishes on red traffic light "brake: 1.0"
	to topic /carla/{}/vehicle_brake_state

for run: roslaunch traffic_lights_controller traffic_lights_controller.launch


