## About
This Package implements a behaviour agent for our autonomous car using __Behaviour Trees__. It uses the _py_trees_ Framework, that works well with ROS. All the dependencies for that library should be included in the installation instructions (wiki). For visualization at runtime you might want to also install this [rqt-Plugin](https://wiki.ros.org/rqt_py_trees). 

## Tree Definition
The tree is defined in the `grow_a_tree()`-function inside `src/behavior_agent/behavior_tree.py`, which is also the main node. It can be visualized using an [rqt-Plugin](https://wiki.ros.org/rqt_py_trees). This is also the Place to change the execution rate of the tree: 
``` python
...
behaviour_tree.tick_tock(500)
...
```

## Behaviours
_Behaviours_ are implemented in the `src/behavior_agent/behaviours/` directory. All the behaviours used in the current Version of the tree are contained as skeletons. 
#### Blackboard
To deal with the asynchronity of ROS, all the Topics this Tree subscribes to, should be written to the Blackboard at the beginning of each tick. I wrote a Node, that automates this Task. Just add your Node to the list in `src/behavior_agent/behaviours/topics2blackboard.py`:
``` python
...
topics =[
    {'name':f"/carla/{role_name}/odometry", 'msg':Odometry, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
    ...
    ]
...
```
After that you can access them from everywhere in your Behaviour-Code using:
``` python
...
self.blackboard = py_trees.blackboard.Blackboard()
...
odo = self.blackboard.get("/carla/ego_vehicle/odometry")
...
```
Note that you still need to resolve the data-fields of the message (i.e. `blackboardmessage.data` for a `Float64`).

### Guidelines
When implementing new behaviours you should adhere to the following guidelines:

#### Non-Blocking
You should avoid doing complicated calculations inside the behaviours. Use asynchronous Callbacks instead, and return ```RUNNING``` while another Node does the computing. 

Generally Conditions should never return ```RUNNING``` and Action-Behaviours should only return ```FAILURE``` in special cases. 

#### Functions
Behaviours generally provide five Functions (you can define more of course). Short Explanation when they get called and how to use them:
##### `__init__()`:
You should probably never use this.
##### `setup()`:
Gets called whenever the tree gets set up for the first time. Use this to setup local variables that dont need to change, like ```self.blackboard = py_trees.blackboard.Blackboard()``` or Middleware like ROS-Publishers (Subscribers should be setup using the method mentioned above).
##### `initialise()`:
Gets called everytime the Behaviour is entered for a new execution. Add Code that only needs to called once at the beginning of a behaviour (i.e. publishing a new target speed).
##### `update()`:
Main Function of a behaviour, that gets called everytime the behaviour is ticked. Here you need to return ```SUCCESS```, ```RUNNING``` or ```FAILURE```.
##### `terminate()`:
This gets called, whenever a behaviour is cancelled by a higher priority branch. Use to terminate Middleware-Connections or Asynchronous Calculations, whose Results are not needed anymore. 


## Authors
Valentin Höpfner