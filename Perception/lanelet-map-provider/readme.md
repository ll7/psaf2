## About

Reads opendrive map string from /carla/WorldInfo, converts to openstreetmap, from that to lanelet2 and saves as binary.
The path to the binary is then published in /psaf/LaneletMap.

## How to get lanelet map object?
```python
import rospy
import lanelet2
from carla_custom_msgs.msg import LaneletMap

# ...
# subscribe to topic
self.sub = rospy.Subscriber("/psaf/lanelet_map", LaneletMap, self.callback)

# ...
def callback(self, lanelet_msg):
    lanelet_map = lanelet2.io.load(lanelet_msg.lanelet_bin_path)

```

## How to launch this node?
```shell
roslaunch lanelet-map-provider lanelet-map-provider-example.launch
```

