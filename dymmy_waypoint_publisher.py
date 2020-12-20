import rospy
import string
import math
import time
import sys

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class WayPoints:
    def __init__(self):
       
        self.pub = rospy.Publisher('/carla/ego_vehicle/waypoints', PoseStamped, queue_size=10)   
        self.goalMsg = PoseStamped()
        # params & variables
        #self.goalMsg.header.frame_id = 0
        self.goalMsg.pose.position.x = 61.4
        self.goalMsg.pose.position.y = -10.62
        self.goalMsg.pose.position.z = 0.05
        self.goalMsg.pose.orientation.z = 0.0
        self.goalMsg.pose.orientation.w = 1.0
        # Publish the first point
        time.sleep(1)
        self.goalMsg.header.stamp = rospy.Time.now()
        self.pub.publish(self.goalMsg) 
        rospy.loginfo("Initial point published!") 
        #self.goalId = self.goalId + 1 
        for i in range(10):
            self.goalMsg.pose.position.x =self.goalMsg.pose.position.x - 2
            self.pub.publish(self.goalMsg) 
            rospy.loginfo("Next point published!") 




if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('waypoint_path', anonymous=True)
        mg = WayPoints()   
        #rospy.errinfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")
