from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from custom_carla_msgs.msg import PerceptionInfo
from time import sleep
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import pathlib
import sys
import traceback

darknet_path = str(pathlib.Path(__file__).parent.parent.parent.absolute() / 'darknet')
sys.path.append(darknet_path)
from darknet_carla import YOLO
from darknet import print_detections


class StreetSignDetector:
    """
    Class to use /carla/<role_name>/semantic_segmentation_front/image and /carla/<role_name>/rgb_front/image topic
    to detect street signs and publish them to /psaf/<role_name>/perception_info and /psaf/<role_name>/speed_limit topics.
    """

    def __init__(self, role_name):
        self.role_name = role_name
        self.bridge = CvBridge()
        self.yolo_detector = YOLO()

        # CAMERA RESOLUTIONS
        # TODO adjustable resolutions (or even better, get resolution from topic)
        self.rgb_img = np.zeros((600, 800, 3), np.uint8)
        self.semantic_segmentation_img = np.zeros((300, 400, 3), np.uint8)

        # COLOR SETTINGS (B, R, G)
        self.semantic_street_signs_color = [ 0, 220, 220 ]
        self.semantic_traffic_light_color = [ 30, 170, 250 ]
        self.semantic_road_line_color = [ 50, 234, 157 ]        # stop sign on street
        self.semantic_replace_by_color = [ 255, 255, 255 ]      # sign's color to be (temporarily) replaced by

        # array of colors that should be extracted in blocks
        self.semantic_segment_search_colors = [ self.semantic_street_signs_color, self.semantic_traffic_light_color ]

        # FOR DEBUGGING 
        self.debug = False                      # enables debug messages and saving of found street signs
        self.global_filename_counter = 0        # debug image file counter
        self.filename_prefix = ""               # prefix for street sign images

        # SUBSCRIBERS
        self._semantic_segmentation_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/semantic_segmentation_front/image".format(role_name), Image, self.semantic_segmentation_image_updated)

        self._rgb_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/rgb_front/image".format(role_name), Image, self.rgb_image_updated)
            

        # PUBLISHERS
        self.speed_limit_publisher = rospy.Publisher(
            "/psaf/{}/speed_limit".format(self.role_name), Float64, queue_size=1, latch=True)

        self.perception_info_publisher = rospy.Publisher(
            "/psaf/{}/perception_info".format(self.role_name), PerceptionInfo, queue_size=1, latch=True)


    def rgb_image_updated(self, data):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print_exc()


    def semantic_segmentation_image_updated(self, data):
        try:
            self.semantic_segmentation_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print_exc()


    def detect_street_signs(self, img_rgb, img_sem):
        '''
        Searches for street signs in the semantic segmentation (img_sem)
        if present, calls get_darknet_detection on the (whole) RGB camera image to get their detections

        :param img_rgb: The RGB camera image
        :param img_sem: The semantic segmentation camera image (by now its resolution must be 0.5 times the RGB camera's resolution)
        :return: If possible, the detection as an instance of PerceptionInfo; None otherwise
        '''

        height, width, channels = img_sem.shape

        street_sign_counter = 0

        # search for the next pixel of a street sign
        for h in range(height):
            for w in range(width):
                # if one pixel's color is in semantic_segment_search_colors, get the whole block
                if img_sem[h, w] in self.semantic_segment_search_colors:
                    color_found = img_sem[h, w]
                    img_sem = self.get_block_by_pixel(img_rgb, img_sem, color_found, h, w, height, width, self.filename_prefix + str(street_sign_counter))

                    if img_sem != []: # valid result? 
                        filename_counter += 1

        # if there have been found any pixel blocks of interest, call darknet and return its detection
        if street_sign_counter > 0:
            darknet_detection = self.get_darknet_detection(img_rgb)
            return darknet_detection

        # or return None otherwise
        return None


    def get_block_by_pixel(self, img_rgb, img_sem, search_color, h_current, w_current, height, width, filename):
        '''
        Finds the street sign in the image of the segmentation camera which includes the pixel at position (h_current,
        w_current), cuts the shape out of the rgb camera image and saves the street sign in a file.

        :param img_rgb: current image of the rgb camera
        :param img_sem: current image of the semantic segmentation camera
        :param search_color: the color searched for (BRG!), e.g. [ 0, 220, 220 ]
        :param h_current: x position of the currently observed pixel
        :param w_current: y position of the currently observed pixel
        :param height: height of img_sem_seg
        :param width: width of img_sem_seg
        :param filename: filename for the next street sign
        :return: the extracted img_sem part in which the latest found street sign is contained
        '''

        pixels = []
        current_pixels = []
        pixels.append([h_current, w_current])
        current_pixels.append([h_current, w_current])
        img_sem[h_current, w_current, 2] = 0

        # TODO adjustable threshold ? 

        while len(current_pixels) > 0:
            for pixel in current_pixels:

                # check if pixel above is part of the street sign
                if pixel[0] > 0 and img_sem[pixel[0] - 1, pixel[1]] == search_color:
                    current_pixels.append([pixel[0] - 1, pixel[1]])
                    pixels.append([pixel[0] - 1, pixel[1]])
                    img_sem[pixel[0] - 1, pixel[1]] = self.semantic_replace_by_color

                # check if pixel below is part of the street sign
                if pixel[0] < height - 1 and img_sem[pixel[0] + 1, pixel[1]] == search_color:
                    current_pixels.append([pixel[0] + 1, pixel[1]])
                    pixels.append([pixel[0] + 1, pixel[1]])
                    img_sem[pixel[0] + 1, pixel[1]] = self.semantic_replace_by_color

                # check if pixel to the left is part of the street sign
                if pixel[1] > 0 and img_sem[pixel[0], pixel[1] - 1] == search_color:
                    current_pixels.append([pixel[0], pixel[1] - 1])
                    pixels.append([pixel[0], pixel[1] - 1])
                    img_sem[pixel[0], pixel[1] - 1] = self.semantic_replace_by_color

                # check if pixel to the right is part of the street sign
                if pixel[1] < width - 1 and img_sem[pixel[0], pixel[1] + 1] == search_color:
                    current_pixels.append([pixel[0], pixel[1] + 1])
                    pixels.append([pixel[0], pixel[1] + 1])
                    img_sem[pixel[0], pixel[1] + 1] = self.semantic_replace_by_color
                current_pixels.remove(pixel)

        # ignore too small images, return empty array
        if(len(pixels) < 50):
            return []

        # calculate offset of the street sign
        min_height = 999999
        max_height = 0
        min_width = 999999
        max_width = 0

        for pixel in pixels:
            if pixel[0] < min_height:
                min_height = pixel[0]

            if pixel[0] > max_height:
                max_height = pixel[0]

            if pixel[1] < min_width:
                min_width = pixel[1]

            if pixel[1] > max_width:
                max_width = pixel[1]

        new_height = max_height - min_height
        new_width = max_width - min_width

        # create new image for the detected street sign
        street_sign_image = np.zeros((new_height * 2 + 2, new_width * 2 + 2, 3), np.uint8)
        street_sign_image[:, :] = (255, 255, 255)

        # TODO adjustable resolutions?

        # map every pixel of the semantic segmentation image to a 2x2 part of the rgb image (because the rgb
        # resolution is two times the resolution of the semantic segmentation camera)
        for pixel in pixels:
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2, 0] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2), 0]
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2, 1] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2), 1]
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2, 2] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2), 0]

            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2 + 1, 0] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2 + 1), 2]
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2 + 1, 1] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2 + 1), 1]
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2 + 1, 2] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2 + 1), 2]

            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2, 0] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2), 2]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2, 1] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2), 1]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2, 2] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2), 0]

            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2 + 1, 2] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2 + 1), 0]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2 + 1, 1] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2 + 1), 1]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2 + 1, 0] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2 + 1), 2]

        rospy.loginfo("New street sign detected")

        if self.debug:
            street_sign_image = cv2.cvtColor(street_sign_image, cv2.COLOR_BGR2RGB)
            cv2.imwrite(str(pathlib.Path(__file__).parent.absolute()) + '/' + filename + '_' + str(self.global_filename_counter) + '.png', street_sign_image)
            self.global_filename_counter += 1

        return img_sem


    def get_darknet_detection(self, img_rgb):
        '''
        Calls the yolo detector to detect objects in the given RGB camera image.

        :param img_rgb: The full image of the RGB camera
        :return: If possible, the detection with the highest confidence as an PerceptionInfo instance; None otherwise
        '''

        detection = PerceptionInfo()

        try:            
            # call yolo darknet detector
            detection_list, img_detected = self.yolo_detector.detect(img_rgb)
            if self.debug:
#                cv2.imshow('Demo', img_detected)
#                cv2.waitKey(0)
                print("list of detections: ")
                print(*detection_list, sep='\n')

            # map detection_list (sorted ascending by confidence) to PerceptionInfo instance
            # thus, at the moment, only the last detection of detection_list is used (i.e. highest confidence)
            for detection_element in detection_list:
                # detection_element[0] is the label (or class name), e.g. traffic_sign_90, stop or traffic_light_red

                # stop sign
                if detection_element[0] == 'stop':
                    detection.values = ['stop']
                    detection.objects = ['sign']

                # traffic lights
                elif detection_element[0].startswith('traffic_light'):
                    if self.debug:
                        print("traffic light recognized: ", detection_element[0])

                # speed limits
                else:
                    '''
                    split and replace class name characters, e.g. traffic_sign_90 becomes traffic sign 90
                    where values gets assigned the last split (90) and objects gets assigned all splits except the last,
                    joined together by a blank space (i.e. traffic sign / traffic light)
                    '''
                    detection.values = [detection_element[0].split('_')[-1]]
                    detection.poses = []
                    detection.objects = [' '.join(detection_element[0].split('_')[:-1])]
                    
                    if detection.objects[0] == 'traffic sign' and (detection.values[0] == '90'
                                             or detection.values[0] == '60'
                                             or detection.values[0] == '30'):
                        detection.objects = ['speed limit']
                        
                    if detection.objects[0] == 'traffic sign limit':
                        detection.objects = ['speed limit']

                if self.debug:
#                    print('\n====================Perception====================')
#                    print('detection objects: ', self.detection.objects, self.detection.values)
                    pass

            if self.debug:
                # darknet's own print function
                print_detections(detection_list)
            
        except Exception as e:
            if self.debug:
                traceback.print_exc()
            detection = None
            pass

        finally:
            return detection


    def publish_detection(self, detection):
        '''
        Publish the detection to the relevant topic(s)

        :param detection: Detection as an instance of PerceptionInfo
        :return:
        '''

        if not type(detection) is PerceptionInfo or not detection:
            return None

        # if detection actually has a value and is of type PerceptionInfo
        self.perception_info_publisher.publish(detection)
        if self.debug:
            print("published perception info")

        # if detection has numeric values, these are the recognized speed limits
        if detection.values and detection.values[0].isnumeric():
            speed_limit = float(detection.values[0])
            self.speed_limit_publisher.publish(speed_limit)
            if self.debug:
                print("published speed limit")


    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            street_sign_detections = self.detect_street_signs(self.rgb_img, self.semantic_segmentation_img)

            if street_sign_detections:
                self.publish_detection(street_sign_detections) 
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass


def main():
    rospy.init_node('street_sign_detector', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    detector = StreetSignDetector(role_name)

    try:
        detector.run()
    finally:
        del detector

if __name__ == "__main__":
    main()
