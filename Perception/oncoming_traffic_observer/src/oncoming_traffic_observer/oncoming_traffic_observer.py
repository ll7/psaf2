import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from custom_carla_msgs.msg import OncomingTrafficObserver as OncomingTrafficObserverMsg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import carla
import numpy as np
from traceback import print_exc
import time


class OncomingTrafficObserver:
    """
    Class to subscribe /carla/WorldInfo topic and create /psaf/LaneletMap topic.
    """
    
    def __init__(self, role_name):
        self.role_name = role_name
        self.bridge = CvBridge()
        self.semantic_segmentation_image = None
        self.depth_image = None
        
        self.vehicles_color = np.array([ 142, 0, 0 ], np.uint8)
        
        self.last_min_distance = None
        self.last_min_distance_threshold = 1    # threshold in m/s
        self.last_min_distance_time = None
        
        # SUBSCRIBERS
        self._semantic_segmentation_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/front/image_segmentation".format(role_name), Image, self.semantic_segmentation_image_updated)
            
        self._depth_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/depth/front/image_depth".format(role_name), Image, self.depth_image_updated)

        # PUBLISHERS
        self.oncoming_traffic_publisher = rospy.Publisher(
            "/psaf/{}/oncoming_traffic".format(self.role_name), OncomingTrafficObserverMsg, queue_size=1, latch=True)


    def semantic_segmentation_image_updated(self, data):
        try:
            self.semantic_segmentation_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("sem", self.semantic_segmentation_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print_exc()
            
     
    def depth_image_updated(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            #cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            self.depth_image = cv_image_array
            #cv2.imshow("depth", cv_image_array)
            #cv2.waitKey(1)
        except CvBridgeError as e:
            print_exc()
            
            
    def detect_oncoming_traffic(self, img_sem, img_dep):
        if img_sem is None or img_dep is None:
            return (None, None)

        min_distance = None
        approximation_velocity = 0
	
        height, width, channels = img_sem.shape
        for h in range(int(height * 0.25), height):
            for w in range(int(width * 0.45)):
                if np.array_equal(img_sem[h, w], self.vehicles_color):
                    depth_pixel = img_dep[h, w]
                    #normalized = (depth_pixel[2] + depth_pixel[1] * 256 + depth_pixel[0] * 256 * 256) / (256 * 256 * 256 - 1)
                    #in_meters = 1000 * normalized
                    if min_distance is None:
                        min_distance = depth_pixel
                    else:
                        min_distance = min(min_distance, depth_pixel)
        
        if min_distance != None and self.last_min_distance_time != None and self.last_min_distance != None and self.last_min_distance > 0 and min_distance > 0:
            carla.Timestamp
            time_diff = (get_epochtime_ms() - self.last_min_distance_time) / 1000
            distance_diff = self.last_min_distance - min_distance
            approximation_velocity = distance_diff / time_diff
            
        if min_distance is None:
            min_distance = 0
            
        self.last_min_distance_time = get_epochtime_ms()
        self.last_min_distance = min_distance 
            
        return (min_distance, approximation_velocity)
        
                    
    def publish_distance(self, message):
        self.oncoming_traffic_publisher.publish(message)


    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            (distance, approximation_velocity) = self.detect_oncoming_traffic(self.semantic_segmentation_image, self.depth_image)
            
            #publish result
            if distance != None and approximation_velocity != None:
                message = OncomingTrafficObserverMsg()
                message.distance = float(distance)
                message.approximation_velocity = approximation_velocity
                
                self.publish_distance(message) 
                
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass
                
                
def get_epochtime_ms():
    return float(time.time_ns()) // 1_000_000 
                
if __name__ == "__main__":
    rospy.init_node('oncoming_traffic_observer', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    observer = OncomingTrafficObserver(role_name)

    try:
        observer.run()
    finally:
        del observer
        
if __name__ == "__main__":
    main()









