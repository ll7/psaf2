from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import pathlib
from traceback import print_exc
from custom_carla_msgs.srv import DetectSideTraffic

class SideTrafficDetector:
    """
    Class to use '/carla/{}/camera/semantic_segmentation/left/image_segmentation' and '/carla/{}/camera/depth/left/image_depth'
    to get the smallest distance to the nearest car on the left side and 'carla/{}/camera/semantic_segmentation/right/image_segmentation'
    and '/carla/{}/camera/depth/right/image_depth' to get the smallest distance to the nearest car on the right side.
    """

    def __init__(self, role_name):
        self.role_name = role_name
        self.bridge = CvBridge()
        
        self.vehicles_color = np.array([ 142, 0, 0 ], np.uint8)
        
        self.semantic_segmentation_left_img_data = None
        self.semantic_segmentation_right_img_data = None
        self.depth_left_img_data = None
        self.depth_right_img_data = None
        
        # SUBSCRIBERS
        
        self._semantic_segmentation_left_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/left/image_segmentation".format(role_name), Image, self.semantic_segmentation_left_image_updated)
            
        self._semantic_segmentation_right_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/right/image_segmentation".format(role_name), Image, self.semantic_segmentation_right_image_updated)
            
        self._depth_left_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/depth/left/image_depth".format(role_name), Image, self.depth_left_image_updated)
            
        self._depth_right_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/depth/right/image_depth".format(role_name), Image, self.depth_right_image_updated)

       
        
    def semantic_segmentation_left_image_updated(self, data):
        """
        Update callback for the left semantic segmentation camera
        
        :param data: the data received from the topic
        :return:
        """
        try:
            self.semantic_segmentation_left_img_data = data
        except CvBridgeError as e:
            print_exc()
        
            
    def semantic_segmentation_right_image_updated(self, data):
        """
        Update callback for the right semantic segmentation camera
        
        :param data: the data received from the topic
        :return:
        """
        try:
            self.semantic_segmentation_right_img_data = data
        except CvBridgeError as e:
            print_exc()
    
    
    def depth_left_image_updated(self, data):
        """
        Update callback for the left depth camera
        
        :param data: the data received from the topic
        :return:
        """
        try:
            self.depth_left_img_data = data
        except CvBridgeError as e:
            print_exc()
    
    
    def depth_right_image_updated(self, data):
        """
        Update callback for the right depth camera
        
        :param data: the data received from the topic
        :return:
        """
        try:
            self.depth_right_img_data = data
        except CvBridgeError as e:
            print_exc()
        
            
    def detect_side_traffic(self, req):
        """
        Detects all cars on the left or right side of the ego vehicle and calculates the smallest distance to the cars
        
        :param req (left_traffic): True if the left traffic side should be detected; False if the right side should be detected
        :return: returns the smallest distance to the cars on the given side
        """
        if(req.left_traffic):
            if self.semantic_segmentation_left_img_data is None or self.depth_left_img_data is None:
                return 0
            img_sem = self.bridge.imgmsg_to_cv2(self.semantic_segmentation_left_img_data, "bgr8")
            cv_image = self.bridge.imgmsg_to_cv2(self.depth_left_img_data, "32FC1")
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            img_dep = cv_image_array
        else:
            if self.semantic_segmentation_right_img_data is None or self.depth_right_img_data is None:
                return 0
            img_sem = self.bridge.imgmsg_to_cv2(self.semantic_segmentation_right_img_data, "bgr8")
            cv_image = self.bridge.imgmsg_to_cv2(self.depth_right_img_data, "32FC1")
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            img_dep = cv_image_array

        min_distance = None
        depth_car_pixels = None
        height, width, channels = img_sem.shape
        	
        indices = np.where(np.all(img_sem == self.vehicles_color, axis=-1))
        depth_car_pixels = np.take(img_dep, indices)
        if not depth_car_pixels is None and depth_car_pixels.size > 0:
            min_distance = np.min(depth_car_pixels)
        
        if min_distance is None:
            min_distance = 9999
            
        return min_distance


def main():
    rospy.init_node('side_traffic_detector', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    detector = SideTrafficDetector(role_name)
    s = rospy.Service("detect_side_traffic", DetectSideTraffic, detector.detect_side_traffic)  # register ROS-Service
    rospy.spin()

if __name__ == "__main__":
    main()
