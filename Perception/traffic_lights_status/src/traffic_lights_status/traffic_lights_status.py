import numpy as np
import sys
import os
import cv2
import functools
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_carla_msgs.msg import PerceptionInfo
import street_sign_detector
#from street_sign_detector import img_traffic_light_detector

import carla
import rospy


class TrafficLightsDetector():
    """
    The class finds the red traffic light mask(tag) on the image and publishes the status
    """

    def __init__(self, world, role_name):
        self.vehicle = None
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')
        self.world = world

        # ==========================================
        # -- Subscriber to ego vehicle --------------
        # ==========================================

        self._semantic_segmentation_front_image_subscriber = rospy.Subscriber(
            "/psaf/{}/perception_info".format(self.role_name), PerceptionInfo,
            self.traffic_light_status)

        # ==========================================
        # -- Publisher to ego vehicle --------------
        # ==========================================

        # traffic light status
        self.traffic_light_state_publisher = rospy.Publisher(
            "/carla/{}/traffic_light_state".format(role_name), String, queue_size=1)

    def run(self):
        """
        Look for an carla actor with name 'ego_vehicle'
        Changes status on red traffic light
        """
        # rospy.loginfo("Waiting for ego vehicle...")
        for actor in self.world.get_actors():
            while actor.attributes.get('role_name') == self.role_name:
                self.ego_vehicle = actor
            rospy.loginfo("Ego vehicle found.")

    def traffic_light_status(self, perception):
        if perception.objects == traffic_light:
            self.traffic_light_collor()


    def traffic_light_collor(self, perception):
        rospy.loginfo ("Reading Perception Info")
        #img = img_to_tl_detector(image)
        rospy.loginfo ("Reading Image")
        
        ## Read image
        image = cv2.imread('0_117.png')
        #image = street_sign_detector.img_
        img = image
        print(img)

        ## convert to hsv
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define lower and uppper limits of what we call "gray"
        gray_lo = np.array([0, 0, 0])
        gray_hi = np.array([220, 220, 220])
        # Mask image to only select gray
        mask_gray = cv2.inRange(hsv, gray_lo, gray_hi)
        # Change image to red where we found grey
        img[mask_gray > 0] = (0, 0, 0)

        # Define lower and uppper limits of what we call "brown"
        brown_lo = np.array([10, 0, 0])
        brown_hi = np.array([20, 255, 255])
        # Mask image to only select browns
        mask_brown = cv2.inRange(hsv, brown_lo, brown_hi)
        # Change image to red where we found brown
        img[mask_brown > 0] = (0, 0, 0)

        # change everything to black where pixel is white
        white_lo = np.array([0, 0, 168])
        white_hi = np.array([172, 111, 255])
        mask_white = cv2.inRange(hsv, white_lo, white_hi)
        img[mask_white > 0] = (0, 0, 0)

    ################# find green ################
        rows, cols, _ = img.shape
        color_B = 0  # blue
        color_G = 0  # green
        color_R = 0  # red
        color_N = 0  # neutral/gray color

        for i in range(rows):
            for j in range(cols):
                k = img[i, j]
                if k[0] > k[1] and k[0] > k[2]:
                    color_B = color_B + 1
                    continue
                if k[1] > k[0] and k[1] > k[2]:
                    color_G = color_G + 1
                    continue
                if k[2] > k[0] and k[2] > k[1]:
                    color_R = color_R + 1
                    continue
                color_N = color_N + 1
        pix_total = rows * cols
        b = color_B / pix_total
        g = color_G / pix_total
        r = color_R / pix_total
        n = color_N / pix_total

        avg_color_per_row = np.average(img, axis=0)
        avg_color = np.average(avg_color_per_row, axis=0)

        if (g > r) & (g > b):
            rospy.loginfo("Traffic Light GREEN")
            print("Traffic Light GREEN")
            ampel_status = 'GREEN'
            self.traffic_light_state_publisher.publish(ampel_status)

        elif (r > g) & (r > b) & (r > 0.017) & (n < 0.99):
            #if avg_color[0] > 2 and avg_color[1] < 12 and avg_color[2] > 11:
            if avg_color[0] < 9 and avg_color[1] > 7 and avg_color[1] > 6:
                rospy.loginfo("Traffic Light YELLOW")
                print("Traffic Light YELLOW")
                ampel_status = 'YELLOW'
                self.traffic_light_state_publisher.publish(ampel_status)
            else:
                rospy.loginfo("Traffic Light RED")
                print("Traffic Light RED")
                ampel_status = 'RED'
                self.traffic_light_state_publisher.publish(ampel_status)

        else:
            rospy.loginfo("Traffic Light NotIdentified")
            ampel_status = 'NotIdentified'
            self.traffic_light_state_publisher.publish(ampel_status)



            #ampel_status = "YES_TRAFFIC_LIGHT!"
            #rospy.loginfo("Caution, the traffic light!")
            #self.traffic_light_state_publisher.publish(ampel_status)

def main():
    rospy.init_node('brake_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    host = rospy.get_param("host", "127.0.0.1")
    port = rospy.get_param("port", 2000)

    carla_client = carla.Client(host=host, port=port)
    world = carla_client.get_world()
    lt = TrafficLightsDetector(world, role_name)

    try:
        lt.run()
    finally:
        del lt
    rospy.loginfo("Done")


if __name__ == '__main__':
    main()
