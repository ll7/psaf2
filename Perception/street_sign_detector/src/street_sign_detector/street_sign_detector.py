from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image, CameraInfo
from custom_carla_msgs.msg import PerceptionInfo
from time import sleep
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import pathlib
import sys
from traceback import print_exc

darknet_path = str(pathlib.Path(__file__).parent.parent.parent.absolute() / 'darknet')
sys.path.append(darknet_path)
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
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
        
        # an object must be at least of this size in the semantic segmentation camera before it will be passed to further recognition tasks
        self.minimal_object_size_for_detection = 50
        self.extract_padding = 20

        # CAMERA RESOLUTIONS
        # TODO adjustable resolution in get_block_by_pixel()
        self.rgb_img_width = 800
        self.rgb_img_height = 600
        self.rgb_img = np.zeros((self.rgb_img_height, self.rgb_img_width, 3), np.uint8)
        
        self.semantic_segmentation_img_width = 400
        self.semantic_segmentation_img_height = 300
        self.semantic_segmentation_img = np.zeros((self.semantic_segmentation_img_height, self.semantic_segmentation_img_width, 3), np.uint8)
        
        # an object must be at least of this size in the semantic segmentation camera before it will be passed to further recognition tasks
        self.minimal_object_size_for_detection = 50

        # COLOR SETTINGS (B, R, G) (have to be numpy arrays!)
        self.semantic_street_signs_color = np.array([ 0, 220, 220 ], np.uint8)
        self.semantic_traffic_light_color = np.array([ 30, 170, 250 ], np.uint8)
        self.semantic_road_line_color = np.array([ 50, 234, 157 ], np.uint8)        # stop sign on street
        self.semantic_replace_by_color = np.array([ 255, 255, 255 ], np.uint8)      # sign's color to be (temporarily) replaced by

        # array of colors that should be extracted in blocks
        self.semantic_segment_search_colors = [ self.semantic_street_signs_color, self.semantic_traffic_light_color ]

        # FOR DEBUGGING 
        self.debug = True                      # enables debug messages and saving of found street signs
        self.global_filename_counter = 0        # debug image file counter
        self.filename_prefix = ""               # prefix for street sign images


        # SUBSCRIBERS
        self._semantic_segmentation_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/front/image_segmentation".format(role_name), Image, self.semantic_segmentation_image_updated)

        self._rgb_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/front/image_color".format(role_name), Image, self.rgb_image_updated)
            
        self._semantic_segmentation_front_camera_info_subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/front/camera_info".format(role_name), CameraInfo, self.semantic_segmentation_front_camera_info_updated)

        self._rgb_front_camera_info_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/front/camera_info".format(role_name), CameraInfo, self.rgb_front_camera_info_updated)


        # PUBLISHERS
        self.speed_limit_publisher = rospy.Publisher(
            "/psaf/{}/speed_limit".format(self.role_name), Float64, queue_size=1, latch=True)

        self.perception_info_publisher = rospy.Publisher(
            "/psaf/{}/perception_info".format(self.role_name), PerceptionInfo, queue_size=1, latch=True)

        
    def rgb_image_updated(self, data):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.debug:
#                cv2.imwrite(str(pathlib.Path(__file__).parent.absolute()) + '/rgb.png', img)
                cv2.imshow('RGB', self.rgb_img)
                cv2.waitKey(1)
        except CvBridgeError as e:
            print_exc()


    def semantic_segmentation_image_updated(self, data):
        try:
            self.semantic_segmentation_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.semantic_segmentation_img = self.resize_semantic_segmentation_image(self.semantic_segmentation_img, self.rgb_img)
            if self.debug:
#                cv2.imwrite(str(pathlib.Path(__file__).parent.absolute()) + '/semseg.png', img)
                cv2.imshow('Semantic', self.semantic_segmentation_img)
                cv2.waitKey(1)
        except CvBridgeError as e:
            print_exc()
            
            
    def rgb_front_camera_info_updated(self, data):
        try:
            if self.rgb_img_width != data.width or self.rgb_img_height != data.height:
#                print("rgb width old: ", self.rgb_img_width)
#                print("rgb width new: ", data.width)
                self.rgb_img_width = data.width
                self.rgb_img_height = data.height
                self.rgb_img = np.zeros((data.height, data.width, 3), np.uint8)
                
        except Exception as e:
            print_exc()
            raise e
            
            
    def semantic_segmentation_front_camera_info_updated(self, data):
        try:
            if self.semantic_segmentation_img_width != data.width or self.semantic_segmentation_img_height != data.height:
#                print("sem width old: ", self.semantic_segmentation_img_width)
#                print("sem width new: ", data.width)
                self.semantic_segmentation_img_width = data.width
                self.semantic_segmentation_img_height = data.height
                self.semantic_segmentation_img = np.zeros((data.height, data.width, 3), np.uint8)
                
        except Exception as e:
            print_exc()
            raise e


    def resize_semantic_segmentation_image(self, img_sem, img_rgb):
        height_rgb, width_rgb, channels_rgb = img_rgb.shape
        height_sem, width_sem, channels_sem = img_sem.shape
        ratio_height = height_rgb / height_sem
        ratio_width = width_rgb / width_sem
		
		# the relative difference between the heigth of the semantic segmentation camera and the height of the rgb camera is 
		# smaller than the relative difference between the width
        if ratio_height < ratio_width:
			# apply the height of the rgb image to the semantic segmentation image
            img_resized = cv2.resize(img_sem, (int(width_sem * ratio_height), int(height_sem * ratio_height)),
                                     fx=0, fy=0, interpolation=cv2.INTER_NEAREST)
		# the relative difference between the width of the semantic segmentation camera and the width of the rgb camera is 
		# smaller than the relative difference between the heigth
        elif ratio_width < ratio_height:
			# apply the width of the rgb image to the semantic segmentation image
            img_resized = cv2.resize(img_sem, (int(width_sem * ratio_width), int(height_sem * ratio_width)),
                                     fx=0, fy=0, interpolation=cv2.INTER_NEAREST)
        else:
            img_resized = cv2.resize(img_sem, (width_rgb, height_rgb), fx=0, fy=0, interpolation=cv2.INTER_NEAREST)
        height_resized, width_resized, channels_resized = img_resized.shape
        new_img_sem = np.zeros((height_rgb, width_rgb, 3), np.uint8)
        x_offset = int((width_rgb - width_resized) / 2)
        y_offset = int((height_rgb - height_resized) / 2)
		# put black bars on the top and the bottom of the semantic segmentation image (if ratio_width < ratio_height)
		# or on the left and right side (if ratio_width < ratio_height)
        new_img_sem[y_offset:y_offset + height_resized, x_offset:x_offset + width_resized] = img_resized
        return new_img_sem
        

    def detect_street_signs(self, img_rgb, img_sem):
        '''
        Searches for street signs in the semantic segmentation (img_sem)
        if present, calls get_darknet_detection on the (whole) RGB camera image to get their detections

        :param img_rgb: The RGB camera image
        :param img_sem: The semantic segmentation camera image (by now its resolution must be 0.5 times the RGB camera's resolution)
        :return: If possible, the detection as an instance of PerceptionInfo; None otherwise
        '''

        img_sem = self.resize_semantic_segmentation_image(img_sem, img_rgb)
        height, width, channels = img_sem.shape

        street_sign_counter = 0
        traffic_light_counter = 0

#        if self.debug:
#            print("detecting street signs....")
            
        # search for the next pixel of a street sign
        for h in range(height):
            for w in range(width):
                # if one pixel's color is in semantic_segment_search_colors, get the whole block
                try:
                    for search_color in self.semantic_segment_search_colors:
                        if np.array_equal(img_sem[h, w], search_color):
                            color_found = img_sem[h, w].copy()
                            img_sem[h, w] = self.semantic_replace_by_color
                            (street_sign_image, returned_img_sem) = self.get_block_by_pixel(img_rgb, img_sem, color_found, h, w, height, width, self.filename_prefix + str(street_sign_counter))
                            
                            # valid result?
                            if not returned_img_sem is None:
                                img_sem = returned_img_sem
                                if np.array_equal(color_found, self.semantic_street_signs_color):
                                    street_sign_counter += 1
                                if np.array_equal(color_found, self.semantic_traffic_light_color):
                                    traffic_light_counter += 1
                except Exception as e:
                    if self.debug:
                        print_exc()
                    raise e

        # if there have been found any pixel blocks of interest, call darknet and return its detection
        if street_sign_counter > 0:
            if self.debug:
                print("Found %d street signs" % street_sign_counter)
                
            darknet_detection = self.get_darknet_detection(img_rgb)
            return darknet_detection
            
        if traffic_light_counter > 0:
            if self.debug:
                print("Found %d traffic lights" % traffic_light_counter)
                
            # TODO traffic light detection
            
            darknet_detection = self.get_darknet_detection(img_rgb)
            return darknet_detection
            pass

        # TODO combine the detection returns => return an array of PerceptionInfo instances in the end
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
        :return: returns a list of the extracted img_sem part in which the latest found street sign is contained and of the img_sem with all the pixels of the current street sign block converted to semantic_replace_by_color - or it returns None instead, when the block of pixels belonging together is too small and below the configured threshold (e.g. the traffic sign is too far away and therefore too small until now)
        '''

        pixels = []
        current_pixels = []
        pixels.append([h_current, w_current])
        current_pixels.append([h_current, w_current])
        img_sem[h_current, w_current, 2] = 0
        
        # TODO adjustable padding? 
        
#        if self.debug:
#            print("get_block_by_pixel, type of img_rgb and img_sem: ", type(img_rgb), type(img_sem))

        while len(current_pixels) > 0:
            for pixel in current_pixels:
                # check if pixel above is part of the street sign
                if pixel[0] > 0 and np.array_equal(img_sem[pixel[0] - 1, pixel[1]], search_color):
                    current_pixels.append([pixel[0] - 1, pixel[1]])
                    pixels.append([pixel[0] - 1, pixel[1]])
                    img_sem[pixel[0] - 1, pixel[1]] = self.semantic_replace_by_color

                # check if pixel below is part of the street sign
                if pixel[0] < height - 1 and np.array_equal(img_sem[pixel[0] + 1, pixel[1]], search_color):
                    current_pixels.append([pixel[0] + 1, pixel[1]])
                    pixels.append([pixel[0] + 1, pixel[1]])
                    img_sem[pixel[0] + 1, pixel[1]] = self.semantic_replace_by_color

                # check if pixel to the left is part of the street sign
                if pixel[1] > 0 and np.array_equal(img_sem[pixel[0], pixel[1] - 1], search_color):
                    current_pixels.append([pixel[0], pixel[1] - 1])
                    pixels.append([pixel[0], pixel[1] - 1])
                    img_sem[pixel[0], pixel[1] - 1] = self.semantic_replace_by_color

                # check if pixel to the right is part of the street sign
                if pixel[1] < width - 1 and np.array_equal(img_sem[pixel[0], pixel[1] + 1], search_color):
                    current_pixels.append([pixel[0], pixel[1] + 1])
                    pixels.append([pixel[0], pixel[1] + 1])
                    img_sem[pixel[0], pixel[1] + 1] = self.semantic_replace_by_color
                current_pixels.remove(pixel)

        # ignore too small images, return None
        if(len(pixels) < self.minimal_object_size_for_detection):
            return (None, None)

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
        street_sign_image = np.zeros((new_height+1, new_width+1, 3), np.uint8)
        street_sign_image[:, :] = (255, 255, 255)

        # TODO adjustable resolutions?

        # map every pixel of the semantic segmentation image to a 2x2 part of the rgb image (because the rgb
        # resolution is two times the resolution of the semantic segmentation camera)
        for pixel in pixels:
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 0] = img_rgb[
                (pixel[0]), (pixel[1]), 0]
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 1] = img_rgb[
                (pixel[0]), (pixel[1]), 1]
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 2] = img_rgb[
                (pixel[0]), (pixel[1]), 0]

            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 0] = img_rgb[
                (pixel[0]), (pixel[1]), 2]
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 1] = img_rgb[
                (pixel[0]), (pixel[1]), 1]
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 2] = img_rgb[
                (pixel[0]), (pixel[1]), 2]

            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 0] = img_rgb[
                (pixel[0]), (pixel[1]), 2]
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 1] = img_rgb[
                (pixel[0]), (pixel[1]), 1]
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 2] = img_rgb[
                (pixel[0]), (pixel[1]), 0]

            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 2] = img_rgb[
                (pixel[0]), (pixel[1]), 0]
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 1] = img_rgb[
                (pixel[0]), (pixel[1]), 1]
            street_sign_image[(pixel[0] - min_height), (pixel[1] - min_width), 0] = img_rgb[
                (pixel[0]), (pixel[1]), 2]

        rospy.loginfo("New street sign detected")


        # save the cut out street sign to file for debug / training reasons...
        if self.debug:
            street_sign_image = cv2.cvtColor(street_sign_image, cv2.COLOR_BGR2RGB)
            filename_base = str(pathlib.Path(__file__).parent.absolute()) + "/" + filename + "_{}.png"
            
            # ... without overwriting anything ...
            while pathlib.Path(filename_base.format(str(self.global_filename_counter))).is_file():
                self.global_filename_counter += 1
            filename_full = filename_base.format(str(self.global_filename_counter))
                
            cv2.imwrite(filename_full, street_sign_image)
            self.global_filename_counter += 1

        return (street_sign_image, img_sem)


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
            # TODO ?
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

        try:
            if not type(detection) is PerceptionInfo or not detection or not detection.objects:
                return None
            else:
                if self.debug:
                    print("detection: ", detection)
        except Exception as e:
            if self.debug:
                print("detection: ", detection)
                print("type of detection: ", type(detection)) 
                traceback.print_exc()
            pass
            

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
        r = rospy.Rate(2)
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