## About

Waits unitl it received lanelet map, gnss signal, odometry signal and a target, then computes global path to target.
The path is published to /psaf/GlobalPath.

## How to launch this node?
```shell
roslaunch global_planner global_planner_example.launch
```

## How to set target?
target_publisher.py publishes gnss coordinates:
```python
self.target_pub = rospy.Publisher("/psaf/target_point", NavSatFix, queue_size=1, latch=True)
target = NavSatFix()
target.latitude = -0.0016077391132190087
target.longitude = -8.324308136330368e-05
target.altitude = 1.965698003768921
self.target_pub.publish(target)
```

## How to see everything together?
The path can be seen in Rviz and the car and map in CarlaViz.
Use the start script in /scripts:
```shell
cd scripts
./start_script.sh
```