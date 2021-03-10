from std_msgs.msg import Bool, Float64, String
from sensor_msgs.msg import Image, CameraInfo
from custom_carla_msgs.msg import PerceptionInfo
from time import sleep
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import pathlib
import sys
import pytesseract
from traceback import print_exc

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
from traffic_light_recognition import detect_traffic_light_color

class StreetSignDetector:
    """
    Class to use /carla/<role_name>/semantic_segmentation_front/image and /carla/<role_name>/rgb_front/image topic
    to detect street signs and publish them to /psaf/<role_name>/perception_info and /psaf/<role_name>/speed_limit topics.
    """

    def __init__(self, role_name):
        self.role_name = role_name
        self.bridge = CvBridge()
        
        # an object must be at least of this size in the semantic segmentation camera before it will be passed to further recognition tasks
        self.minimal_object_size_for_detection = 15
        self.extract_padding = 20
        
        # OCR settings
        self.tesseract_config = "--psm 11 --oem 1"
        
        self.speed_limit_replacements = [
                                    [ '$', '9' ],
                                    [ '(', '' ],
                                    [ ')', '' ],
                                    [ 'O', '0' ],
                                    [ '{', '' ],
                                    [ '}', '' ]
                                ]
        
        self.rgb_img = None
        self.semantic_segmentation_img = None
        
        # an object must be at least of this size in the semantic segmentation camera before it will be passed to further recognition tasks
        self.minimal_object_size_for_detection = 20

        # COLOR SETTINGS (B, R, G) (have to be numpy arrays!)
        self.semantic_street_signs_color = np.array([ 0, 220, 220 ], np.uint8)
        self.semantic_traffic_light_color = np.array([ 30, 170, 250 ], np.uint8)
        self.semantic_replace_by_color = np.array([ 255, 255, 255 ], np.uint8)      # sign's color to be (temporarily) replaced by
        
        # OFFSET: top, right, bottom, left
        self.semantic_street_signs_offset = [0.2, 0, 0.2, 0.5]
        self.semantic_traffic_light_offset = [0, 0.3, 0.4, 0.3]

        # array of colors that should be extracted in blocks
        self.semantic_segment_search_colors = [ 
                                        self.semantic_street_signs_color,
                                        self.semantic_traffic_light_color
                                    ]

        # SUBSCRIBERS
        self._semantic_segmentation_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/front/image_segmentation".format(role_name), Image, self.semantic_segmentation_image_updated)

        self._rgb_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/front/image_color".format(role_name), Image, self.rgb_image_updated)

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
        if present, calls tesseract to read the speed limit

        :param img_rgb: The RGB camera image
        :param img_sem: The semantic segmentation camera image (by now its resolution must be 0.5 times the RGB camera's resolution)
        :return: If possible, the detection as an instance of PerceptionInfo; None otherwise
        '''

        if img_sem is None or img_rgb is None:
            return

        img_sem = self.resize_semantic_segmentation_image(img_sem, img_rgb)
        height, width, channels = img_sem.shape

        street_sign_counter = 0
        traffic_light_counter = 0
            
        detections = PerceptionInfo()
        detections.objects = []
        detections.values = []
        
        traffic_lights_detected = []
        street_signs_detected = []
        
        stop_img = np.zeros((height, width, channels), np.uint8)
        
        # search for the next pixel of a street sign
        for h in range(height):
            for w in range(width):
                # if one pixel's color is in semantic_segment_search_colors, get the whole block
                try:
                    for search_color in self.semantic_segment_search_colors:
                        if np.array_equal(img_sem[h, w], search_color):
            
                            color_found = img_sem[h, w].copy()
                            img_sem[h, w] = self.semantic_replace_by_color
                            
                                                        
                            if np.array_equal(color_found, self.semantic_street_signs_color):
                                if not self.check_image_offset(h, w, self.semantic_street_signs_offset):
                                    continue
                            if np.array_equal(color_found, self.semantic_traffic_light_color):
                                if not self.check_image_offset(h, w, self.semantic_traffic_light_offset):
                                    continue
                            
                            
                            (street_sign_image, img_sem) = self.get_block_by_pixel(img_rgb, img_sem, color_found, h, w, height, width)
                            
                            # valid result?
                            if not street_sign_image is None:
                                if np.array_equal(color_found, self.semantic_street_signs_color):
                                    street_sign_counter += 1
                                    street_signs_detected.append([ round(w / width, 2), round(h / height, 2), street_sign_image.copy() ])
                                if np.array_equal(color_found, self.semantic_traffic_light_color):
                                    traffic_light_counter += 1
                                    # store relative x and y pos with cut out rgb image
                                    traffic_lights_detected.append([ round(w / width, 2), round(h / height, 2), street_sign_image.copy() ])
                except Exception as e:
                    print_exc()
                    pass
            
        # if there have been found any pixel blocks of interest, call darknet and return its detection
        if street_sign_counter > 0:
            for (x_rel, y_rel, street_sign_rgb) in street_signs_detected:
                gray = cv2.cvtColor(street_sign_rgb, cv2.COLOR_RGB2GRAY)
                resize_to = 100
                preprocess = "tb"
                
                if resize_to != 0:
                    resize_factor = resize_to / gray.shape[1]
                    new_width = int(gray.shape[1] * resize_factor)
                    new_height = int(gray.shape[0] * resize_factor)
                    gray = cv2.resize(gray, (new_width, new_height))
            
                if "b" in preprocess:
                	gray = cv2.medianBlur(gray, 3)
                if "t" in preprocess:
                	gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 1)
                	
                text = pytesseract.image_to_string(gray, config="--psm 13", lang='eng')
                text = text.strip()
                    
                for replacement in self.speed_limit_replacements:
                    text = text.replace(replacement[0], replacement[1])
                recognized_number = ''.join([ i for i in text.split() if i.isdigit() ])
                    
                if len(recognized_number) >= 2 and recognized_number in ['30', '60', '90']:
                    detections.objects.append('speed limit')
                    detections.values.append(recognized_number)
                    detections.relative_x_coord.append(x_rel)
                    detections.relative_y_coord.append(y_rel)
            
        if traffic_light_counter > 0:
            for (x_rel, y_rel, traffic_light_rgb) in traffic_lights_detected:
                traffic_light_color = detect_traffic_light_color(traffic_light_rgb)
                if traffic_light_color != '':
                    detections.objects.append('traffic_light')
                    detections.values.append(traffic_light_color)
                    detections.relative_x_coord.append(x_rel)
                    detections.relative_y_coord.append(y_rel)

        return detections


    def check_image_offset(self, h, w, offset):
        # offset top
        if h < offset[0] * self.semantic_segmentation_img.shape[0]:
            return False
        # offset bottom
        if h > self.semantic_segmentation_img.shape[0] - (offset[2] * self.semantic_segmentation_img.shape[0]):
            return False
        # offset left
        if w < offset[3] * self.semantic_segmentation_img.shape[1]:
            return False
        # offset right
        if w > self.semantic_segmentation_img.shape[1] - (offset[1] * self.semantic_segmentation_img.shape[1]):
            return False
        return True
        
        
    def cut_image_offset(self, image, offset):
        offset_top = round(offset[0] * image.shape[0])
        offset_bottom = image.shape[0] - round(offset[2] * image.shape[0])
        offset_left = round(offset[1] * image.shape[1])
        offset_right = image.shape[1] - round(offset[3] * image.shape[1])
        image_crop = image[offset_top:offset_bottom, offset_left:offset_right].copy()
        return image_crop
        
        
    def get_block_by_pixel(self, img_rgb, img_sem, search_color, h_current, w_current, height, width):
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
        :return: returns a list of the extracted img_sem part in which the latest found street sign is contained and of the img_sem with all the pixels of the current street sign block converted to semantic_replace_by_color - or it returns None instead, when the block of pixels belonging together is too small and below the configured threshold (e.g. the traffic sign is too far away and therefore too small until now)
        '''
            
        pixels = []
        current_pixels = []
        pixels.append([h_current, w_current])
        current_pixels.append([h_current, w_current])
        img_sem[h_current, w_current, 2] = 0

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
            return (None, img_sem)

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

        return (street_sign_image, img_sem)


    def publish_detection(self, detections):
        '''
        Publish the detection to the relevant topic(s)

        :param detection: Detection as an instance of PerceptionInfo
        :return:
        '''

        if not type(detections) is PerceptionInfo or not detections or not detections.objects:
            self.perception_info_publisher.publish(PerceptionInfo())
            return None
            

        # if detection actually has a value and is of type PerceptionInfo
        self.perception_info_publisher.publish(detections)
        
        if detections.objects and type(detections.objects) is list:
            for i in range(len(detections.objects)):
                if detections.objects[i] == "speed limit":
                    speed_limit = float(detections.values[i])
                    self.speed_limit_publisher.publish(speed_limit)


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
