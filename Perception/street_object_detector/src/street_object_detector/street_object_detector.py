import copy

import rospy
import numpy as np
import cv2

from traceback import print_exc
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import String
from carla_msgs.msg import CarlaWorldInfo
from custom_carla_msgs.msg import TrafficLight


class StreetObjectDetector:
    def __init__(self, role_name):
        self.role_name = role_name
        self.bridge = CvBridge()
        self.sem_seg_height_width = (300, 400)

        self.rgb_img = None
        self._semantic_segmentation_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/front/image_segmentation".format(role_name), Image, self.semantic_segmentation_image_updated)

        self.semantic_segmentation_img = None
        self._rgb_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/front/image_color".format(role_name), Image, self.rgb_image_updated)

        self.depth_img = None
        self._rgb_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/depth/front/image_depth".format(role_name), Image, self.depth_image_updated)

        self.traffic_light_publisher = rospy.Publisher(
            "/psaf/{}/traffic_light".format(self.role_name), TrafficLight, queue_size=1, latch=True)

        self.roadmark_publisher = rospy.Publisher("/psaf/{}/traffic_light_img".format(role_name), Image, queue_size=1)

        self.map_number = None
        self.world_info = rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.world_info_received)

        self.mask_right_side = np.zeros(self.sem_seg_height_width, np.uint8)
        self.mask_front_triangle = np.zeros(self.sem_seg_height_width, np.uint8)
        self.create_masks()

    def world_info_received(self, msg):
        self.map_number = int(msg.map_name[-1])

    def rgb_image_updated(self, data):
        """
        Update callback for the RGB camera

        :param data: the data received from the topic
        :return:
        """
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print_exc()

    def semantic_segmentation_image_updated(self, data):
        """
        Update callback for the semantic segmentation camera

        :param data: the data received from the topic
        :return:
        """
        try:
            self.semantic_segmentation_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print_exc()

    def depth_image_updated(self, data):
        """
        Update callback for the RGB camera

        :param data: the data received from the topic
        :return:
        """
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print_exc()

    def create_masks(self):
        """
        Create masks for the segmented segmentation.
        :return:
        """
        height, width = self.sem_seg_height_width

        pts = np.array([[width//2, 0], [width, 0], [width, height], [width//2, height]])  # mask for right side
        cv2.drawContours(self.mask_right_side, [pts], -1, 1, -1, cv2.LINE_AA)

        pts = np.array([[75, 300], [175, 0], [200, 0], [325, 300]])  # mask to detected american style traffic lights
        cv2.drawContours(self.mask_front_triangle, [pts], -1, 1, -1, cv2.LINE_AA)

    def detect_traffic_lights(self):
        sem_img = copy.deepcopy(self.semantic_segmentation_img)
        rgb_img = copy.deepcopy(self.rgb_img)
        depth_img = copy.deepcopy(self.depth_img)

        if self.map_number == 3 or self.map_number == 5:  # on this maps traffic light style is american
            traffic_light_mask = self.mask_front_triangle
        else:
            traffic_light_mask = self.mask_right_side

        try:  # error that can occur at startup
            traffic_light_rgb_mask = np.zeros(self.sem_seg_height_width, np.uint8)  # create mask
            traffic_light_rgb_mask[np.where((sem_img == [30, 170, 250]).all(axis=2))] = 1  # find traffic light
            # use only the traffic lights inside the mask
            traffic_light_rgb_mask = cv2.bitwise_and(traffic_light_rgb_mask, traffic_light_rgb_mask, mask=traffic_light_mask)

            # get distance to closest traffic_light
            depth_img_traffic_lights = cv2.bitwise_and(depth_img, depth_img, mask=traffic_light_rgb_mask)
            non_zero_entries = depth_img_traffic_lights[np.nonzero(depth_img_traffic_lights)]
            if non_zero_entries.size != 0:
                distance_closest_traffic_light = np.min(non_zero_entries)
            else:
                distance_closest_traffic_light = np.inf

            # detect color
            traffic_light_rgb_mask = cv2.resize(traffic_light_rgb_mask, (800, 600))  # upscale mask to fit for rgb image
            traffic_lights_cut_out = cv2.bitwise_and(rgb_img, rgb_img, mask=traffic_light_rgb_mask)  # cut out rgb mask
            color = self.detect_traffic_light_color(traffic_lights_cut_out, 10)

            # publish gathered information
            traffic_light_msg = TrafficLight()
            traffic_light_msg.color = color
            traffic_light_msg.distance = distance_closest_traffic_light
            self.traffic_light_publisher.publish(traffic_light_msg)

        except cv2.error as e:
            print(e.err)
            print(e)
            return

        try:
            im = self.bridge.cv2_to_imgmsg(traffic_lights_cut_out)
            self.roadmark_publisher.publish(im)
        except CvBridgeError as e:
            print(e)

    def detect_traffic_light_color(self, img, pixel_count_threshold):
        """
        Detects the traffic light color by identifying its dominant color

        :param img: BGR image of the traffic light
        :param pixel_count_threshold: Defines the minimum amount of pixels that the assumably dominant color excesses the others
        :return: returns the detected traffic light color as a string (empty if no successful detection)
        """
        ## convert to hsv
        frame_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # red represents two regions in the HSV space
        lower_red = np.array([0, 70, 50], dtype="uint8")
        upper_red = np.array([10, 255, 240], dtype="uint8")

        # red in the range of purple hue
        lower_violet = np.array([170, 70, 50], dtype="uint8")
        upper_violet = np.array([180, 255, 240], dtype="uint8")

        # red mask
        red_mask_orange = cv2.inRange(frame_hsv, lower_red, upper_red)
        red_mask_violet = cv2.inRange(frame_hsv, lower_violet, upper_violet)

        red_mask_full = red_mask_orange + red_mask_violet

        # green
        lower_green = np.array([40, 70, 100], dtype="uint8")
        upper_green = np.array([70, 255, 240], dtype="uint8")
        green_mask = cv2.inRange(frame_hsv, lower_green, upper_green)

        # yellow
        lower_yellow = np.array([20, 93, 0], dtype="uint8")
        upper_yellow = np.array([27, 255, 240], dtype="uint8")
        yellow_mask = cv2.inRange(frame_hsv, lower_yellow, upper_yellow)

        red_count = cv2.countNonZero(red_mask_full)
        green_count = cv2.countNonZero(green_mask)
        yellow_count = cv2.countNonZero(yellow_mask)

        max_count = max(red_count, green_count, yellow_count)

        if max_count < pixel_count_threshold:
            return ''
        elif red_count == max_count:
            return 'red'
        elif green_count == max_count:
            return 'green'
        else:
            return 'yellow'

    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.rgb_img is not None and self.semantic_segmentation_img is not None:
                self.detect_traffic_lights()
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == "__main__":
    rospy.init_node('street_object_detector', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    detector = StreetObjectDetector(role_name)

    try:
        detector.run()
    finally:
        del detector
