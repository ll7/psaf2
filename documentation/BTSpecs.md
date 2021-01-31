# Behaviour Tree - Behaviours Specification
## Collision Avoidance Subtree
******
******
### _Condition:_ No Obstacle Ahead
There should be a function/node that detects obstacles. It should write the information to a topic and this behaviour should query that topic. Additional information (e.g. distance to obstacle) should also be published in the topic (and written to the blackboard).

**OnUpdate:** Return _Success_ if no obstacle ahead, _Failure_ otherwise.

******
### _Action:_ Replan Around Obstacle 
Trigger function for local replaning, set timeout with respect to information published from obstacle detection. 

**OnUpdate:** If timeout but planning not done or error occured, return _Failure_. Before timeout while still planning, return _Running_. If planning finishes in time, return _Success_ and publish new local plan.

**OnTerminate:** When this behaviour is termianted by higher node, cancel the planning (or at least do not publish the result).

## Road Features Subtree - Intersection
******
******
### _Condition:_ Intersection Ahead
Read the information from a topic from Perception.

**OnUpdate:** Return _Success_ if intersection ahead, _Failure_ otherwise.
******

### _Action:_ Approach Intersection 
Slow down and stop at the traffic lights/marker.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

******
### _Action:_ Wait 
Wait for the interscection to be free/traffic lights to become green.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

******
### _Action:_ Enter Intersection 
Enter Intersection.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

******
### _Action:_ Leave Intersection 
Leave Intersection.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

## Road Features Subtree - Roundabout
******
******
### _Condition:_ Roundabout Ahead
Read the information from a topic from Perception.

**OnUpdate:** Return _Success_ if intersection ahead, _Failure_ otherwise.
******

### _Action:_ Approach Roundabout 
Slow down and stop at the roundabout entrance.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

******
### _Action:_ Wait 
Wait for the lane in Roundabout to be free.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

******
### _Action:_ Enter 
Enter Roundabout.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

******
### _Action:_ Leave 
Leave Roundabout. Check for lane switch that needs to be done.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

## Road Features Subtree - Stop
******
******
### _Condition:_ Stop Ahead
Read the information from a topic from Perception. (Stop means traffic ligths, zebra crossing, ...)

**OnUpdate:** Return _Success_ if intersection ahead, _Failure_ otherwise.
******

### _Action:_ Approach 
Slow down and stop at the marker.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

******
### _Action:_ Wait 
Wait for go.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

******
### _Action:_ Go 
Continue.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

## Lane Switch Subtree
***
***
### _Condition:_ Not slowed down by car in front
There should be a function/node that detects cars in front. It should write the information to a topic and this behaviour should query that topic. Additional information (e.g. car speed) should also be published in the topic (and written to the blackboard).

**OnUpdate:** Return _Success_ if no slow car ahead, _Failure_ otherwise.

## Lane Switch Subtree - Multilane
******
***

### _Condition_: Multi-Lane

**OnUpdate:** Return _Success_ if multilane, _Failure_ otherwise.

***
### _Condition_: Left-Lane available

**OnUpdate:** Return _Success_ if left-lane available, _Failure_ otherwise.

***
### _Action_: Wait for left lane free

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

***
### _Action_: Switch to lane left

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

## Lane Switch Subtree - Single-Lane
******

***
### _Condition_: Single-line with dotted line

**OnUpdate:** Return _Success_ if Single-line with dotted line, _Failure_ otherwise.

***
### _Action_: Wait for left lane free

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

***
### _Action_: Switch to lane left

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

***
### _Condition:_ Overtaking possible
Check if overtaking is possible (cars on other lane).

**OnUpdate:** Return _Success_ if possible, _Failure_ otherwise.

***
### _Action_: Overtake

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

***
### _Action_: Switch lane right
Go back to right lane.

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

## Lane Switch Subtree - Right-hand drive

***
***
### _Condition:_ Right lane available

**OnUpdate:** Return _Success_ if right lane available, _Failure_ otherwise.

*** 
### _Condition:_ Not slowed down by car in front
There should be a function/node that detects cars in front. It should write the information to a topic and this behaviour should query that topic. Additional information (e.g. car speed) should also be published in the topic (and written to the blackboard).

**OnUpdate:** Return _Success_ if no slow car ahead, _Failure_ otherwise.

***
### _Action_: wait for lane right free

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.

***
### _Action_: switch to lane right 

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.


## Cruise
******
***
### _Action:_ Cruising.

**OnUpdate:** If ongoing, return _Running_, otherwise _Failure_ or _Success_. 

**OnTerminate:** Cancel this action.