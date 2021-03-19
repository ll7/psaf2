from std_msgs.msg import Bool, Float64, String
from sensor_msgs.msg import Image
from custom_carla_msgs.msg import PerceptionInfo
from time import sleep
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import pathlib
import pytesseract
from traceback import print_exc

class StreetObjectDetector:
    """
    Class to use /carla/<role_name>/semantic_segmentation_front/image and /carla/<role_name>/rgb_front/image topic
    to detect street signs and traffic lights and publish them to /psaf/<role_name>/perception_info and /psaf/<role_name>/speed_limit topics.
    """

    def __init__(self, role_name):
        self.role_name = role_name
        self.bridge = CvBridge()
        
        self.rgb_img = None
        self.semantic_segmentation_img = None
        
        # an object must be at least of this size in the semantic segmentation camera before it will be passed to further recognition tasks
        self.minimal_object_size_for_detection = 2
        
        # OCR settings
        self.ocr_tesseract_configs = [ "--psm 4 --oem 1", "--psm 6 --oem 1", "--psm 9 --oem 1", "--psm 11 --oem 1", "--psm 13 --oem 1" ]
        self.ocr_preprocess_tasks = "tb" # t for thresholding, b for blurring
        self.ocr_resize_to = 100 # ocr resizes input image to that size
        
        # some letters are not detected correctly by tesseract, so replace them accordingly
        self.speed_limit_replacements = [
                                    [ '$', '9' ],
                                    [ '(', '' ],
                                    [ ')', '' ],
                                    [ 'O', '0' ],
                                    [ 'o', '0' ],
                                    [ 'q', '9' ],
                                    [ 'Q', '9' ],
                                    [ '{', '' ],
                                    [ '}', '' ],
                                    [ '8', '9' ]
                                ]
        self.speed_limit_values = [ '30', '60', '90' ]
        
        # minimum count of pixels of the dominant color for traffic light color detection
        self.traffic_light_pixel_count_threshold = 2

        # COLOR SETTINGS (B, R, G) (have to be numpy arrays!)
        self.semantic_street_signs_color = np.array([ 0, 220, 220 ], np.uint8)
        self.semantic_traffic_light_color = np.array([ 30, 170, 250 ], np.uint8)
        self.semantic_replace_by_color = np.array([ 255, 255, 255 ], np.uint8)      # sign's color to be (temporarily) replaced by
        
        # There are dead areas in the camera's field of view where street signs and traffic lights can be ignored
        # OFFSET: [top, right, bottom, left]
        self.semantic_street_signs_offset = [0, 0, 0, 0.6]
        self.semantic_traffic_light_offset = [0, 0.4, 0.5, 0.4]
        
        # array of offsets
        self.semantic_segment_search_offsets = {
                                        'street_signs': self.semantic_street_signs_offset,
                                        'traffic_lights': self.semantic_traffic_light_offset
                                    }

        # array of colors that should be extracted in blocks
        self.semantic_segment_search_colors = {
                                        'street_signs': self.semantic_street_signs_color,
                                        'traffic_lights': self.semantic_traffic_light_color
                                    }

        # SUBSCRIBERS
        self._semantic_segmentation_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/front_flat/image_segmentation".format(role_name), Image, self.semantic_segmentation_image_updated)

        self._rgb_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/front/image_color".format(role_name), Image, self.rgb_image_updated)

        # PUBLISHERS
        self.speed_limit_publisher = rospy.Publisher(
            "/psaf/{}/speed_limit".format(self.role_name), Float64, queue_size=1, latch=True)
            
        self.traffic_light_publisher = rospy.Publisher(
            "/psaf/{}/traffic_light".format(self.role_name), String, queue_size=1, latch=True)

        self.perception_info_publisher = rospy.Publisher(
            "/psaf/{}/perception_info".format(self.role_name), PerceptionInfo, queue_size=1, latch=True)

        
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
        

    def detect_street_objects(self, img_rgb, img_sem):
        """
        Searches for street objects in the semantic segmentation (img_sem) if present, calls tesseract
        to read the speed limit of steet signs or calls the traffic light detection for traffic lights

        :param img_rgb: The RGB camera image
        :param img_sem: The semantic segmentation camera image
        :return: The detections as an instance of PerceptionInfo
        """

        sem_height, sem_width, sem_channels = img_sem.shape
            
        detections = PerceptionInfo()
        detections.objects = []
        detections.values = []
        
        rgb_height, rgb_width, rgb_channels = img_rgb.shape
        rgb_sem_scale_factor = rgb_width / sem_width
        border_height = int((rgb_height - int(sem_height * rgb_sem_scale_factor)) / 2) # in case img_sem and img_rgb are of different height, calculate the offset to add to the following y coordinates
        
        street_objects = {}
        
        # iterate through the defined colors to search for
        for color_key in self.semantic_segment_search_colors.keys():
            color = self.semantic_segment_search_colors[color_key]
            offset = self.semantic_segment_search_offsets[color_key]
            street_objects[color_key] = { "count": 0, "objects": [] }
            
            
            for h in range(int(offset[0] * sem_height), sem_height - int(offset[2] * sem_height)):
                for w in range(int(offset[3] * sem_width), sem_width - int(offset[1] * sem_width)):
                    try:
                        if np.array_equal(img_sem[h, w], color):
                        
                        
                            # extract the pixels that form a block of that color
                            (street_object_image, img_sem, x_rel, y_rel) = self.get_color_block(img_rgb, img_sem, color, h, w, rgb_sem_scale_factor, border_height)
                            
                            # valid result?
                            if not street_object_image is None:
                            	street_objects[color_key]["count"] += 1
                            	street_objects[color_key]["objects"].append([ x_rel, y_rel, street_object_image.copy() ])
                                    
                    except Exception as e:
                        print_exc()
                        pass
            
                        
            # if there have been found any pixel blocks of interest, return its detection
            if street_objects[color_key]["count"] > 0:
            
                if color_key == "street_signs":
                    for (x_rel, y_rel, street_sign_rgb) in street_objects[color_key]["objects"]:
                        ocr_texts = []
                        
                        # execute all defined tesseract configurations and yield their returned values
                        for tesseract_config in self.ocr_tesseract_configs:
                            ocr_texts.append(self.get_ocr_text(street_sign_rgb, tesseract_config, self.ocr_preprocess_tasks, self.ocr_resize_to, self.speed_limit_replacements))
                    
                        recognized_speed_limit = None
                        
                        for ocr_text in ocr_texts:
                            # cut out non-digit characters
                            recognized_number = ''.join([ i for i in ocr_text.split() if i.isdigit() ])
                            
                            # search for valid values and apply the minimum speed limit
                            if recognized_number in self.speed_limit_values and (recognized_speed_limit is None or int(recognized_number) < int(recognized_speed_limit)):
                                recognized_speed_limit = recognized_number
                                
                            
                        if recognized_speed_limit:
                            detections.objects.append(color_key)
                            detections.values.append(recognized_speed_limit)
                            detections.relative_x_coord.append(x_rel)
                            detections.relative_y_coord.append(y_rel)
            
                if color_key == "traffic_lights":
                    for (x_rel, y_rel, traffic_light_rgb) in street_objects[color_key]["objects"]:
                        traffic_light_color = self.detect_traffic_light_color(traffic_light_rgb, self.traffic_light_pixel_count_threshold)
                        
                        if traffic_light_color != '':
                            detections.objects.append(color_key)
                            detections.values.append(traffic_light_color)
                            detections.relative_x_coord.append(x_rel)
                            detections.relative_y_coord.append(y_rel)

        return detections
        
        
    def get_ocr_text(self, street_sign_rgb, tesseract_config, preprocess_tasks, resize_to, replacement_rules):
        """
        Extracts text from a given image by calling pytesseract.
        
        :param street_sign_rgb: the image to extract text from
        :param tesseract_config: the tesseract config string, e.g. "--psm 13"
        :param preprocess_tasks: which preprocess tasks should be executed, possible values: "t" for thresholding, "b" for blurring (or both)
        :param resize_to: resize the preprocessed image to a specific size before calling the OCR
        :param replacement_rules: a dictionary of symbols to be replaced by another symbol, e.g. { "$": "9" }
        :return: the extracted text
        """
        
        gray = cv2.cvtColor(street_sign_rgb, cv2.COLOR_BGR2GRAY)
        
        if resize_to != 0:
            resize_factor = resize_to / gray.shape[1]
            new_width = int(gray.shape[1] * resize_factor)
            new_height = int(gray.shape[0] * resize_factor)
            gray = cv2.resize(gray, (new_width, new_height))

        if "b" in preprocess_tasks:
        	gray = cv2.medianBlur(gray, 3)  # apply blurring
        if "t" in preprocess_tasks:
        	gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 1) # apply thresholding
        	
        text = pytesseract.image_to_string(gray, config=tesseract_config, lang='eng')
        text = text.strip() # cut off whitespace
        
        for replacement in replacement_rules: # replace unwanted characters
            text = text.replace(replacement[0], replacement[1])
        
        return text
    
        
    def get_color_block(self, img_rgb, img_sem, search_color, h_current, w_current, rgb_sem_scale_factor, border_height):
        """
        Finds the object in the image of the segmentation camera with the given color and which includes the pixel at position (h_current,
        w_current), cuts the shape out of the rgb camera image and returns the cut out object

        :param img_rgb: current image of the RGB camera
        :param img_sem: current image of the semantic segmentation camera
        :param search_color: the color searched for (BRG!), e.g. [ 0, 220, 220 ]
        :param h_current: x position of the currently observed pixel
        :param w_current: y position of the currently observed pixel
        :param rgb_sem_scale_factor: The scale factor of the semantic segmentation image (i.e. the factor between rgb width and semantic segmentation width)
        :param border_height: offset between img_rgb and img_sem (add this value to the img_rgb y position)
        :return: returns a list of the extracted img_sem part in which the latest found object is contained and of the img_sem with
        all the pixels of the current object block converted to semantic_replace_by_color - or it returns None for the extracted
        img_sem part instead, when the block of pixels belonging together is too small and below the configured threshold (e.g. the
        object is too far away and therefore too small until now)
        The return list also contains the relative x and y positions in the RGB image as third and fourth elements
        """
            
        pixels = np.empty((0, 2), int)
        current_pixels = np.empty((0, 2), int)
        pixels = np.append(pixels, np.array([ [h_current, w_current] ]), axis=0)
        current_pixels = np.append(current_pixels, np.array([ [h_current, w_current] ]), axis=0)
        (height, width, dimensions) = img_sem.shape
        (rgb_height, rgb_width, rgb_dimensions) = img_rgb.shape
        
        img_sem[h_current, w_current] = self.semantic_replace_by_color

        while current_pixels.size > 0:
            
            for pixel in current_pixels:
            
                # check if pixel above is part of the street sign
                if pixel[0] > 0 and np.array_equal(img_sem[pixel[0] - 1, pixel[1]], search_color):
                
                    current_pixels = np.append(current_pixels, np.array([ [ pixel[0] - 1, pixel[1] ] ]), axis=0)
                    pixels = np.append(pixels, np.array([ [ pixel[0] - 1, pixel[1] ] ]), axis=0)
                    
                    img_sem[pixel[0] - 1, pixel[1]] = self.semantic_replace_by_color

                # check if pixel below is part of the street sign
                if pixel[0] < height - 1 and np.array_equal(img_sem[pixel[0] + 1, pixel[1]], search_color):
                
                    current_pixels = np.append(current_pixels, np.array([ [pixel[0] + 1, pixel[1]] ]), axis=0)
                    pixels = np.append(pixels, np.array([ [pixel[0] + 1, pixel[1]] ]), axis=0)
                    
                    img_sem[pixel[0] + 1, pixel[1]] = self.semantic_replace_by_color

                # check if pixel to the left is part of the street sign
                if pixel[1] > 0 and np.array_equal(img_sem[pixel[0], pixel[1] - 1], search_color):
                
                    current_pixels = np.append(current_pixels, np.array([ [pixel[0], pixel[1] - 1] ]), axis=0)
                    pixels = np.append(pixels, np.array([ [pixel[0], pixel[1] - 1] ]), axis=0)
                    
                    img_sem[pixel[0], pixel[1] - 1] = self.semantic_replace_by_color

                # check if pixel to the right is part of the street sign
                if pixel[1] < width - 1 and np.array_equal(img_sem[pixel[0], pixel[1] + 1], search_color):
                
                    current_pixels = np.append(current_pixels, np.array([ [pixel[0], pixel[1] + 1] ]), axis=0)
                    pixels = np.append(pixels, np.array([ [pixel[0], pixel[1] + 1] ]), axis=0)
                    
                    img_sem[pixel[0], pixel[1] + 1] = self.semantic_replace_by_color#
                    
                current_pixels = np.delete(current_pixels, np.where(np.all(current_pixels == pixel, axis=1)), axis=0)#


        # ignore too small images, return None for the extracted img_sem part
        if(pixels.size < self.minimal_object_size_for_detection):
            return (None, img_sem)

        min_x = int(np.min(pixels, axis=0)[1] * rgb_sem_scale_factor)
        max_x = int(np.max(pixels, axis=0)[1] * rgb_sem_scale_factor)
        
        min_y = int(h_current * rgb_sem_scale_factor) + border_height # the first pixel is in the most upper row
        max_y = int(np.max(pixels, axis=0)[0] * rgb_sem_scale_factor) + border_height
        
        street_object_image = img_rgb[ min_y:max_y, min_x:max_x, :]
        
        if(street_object_image.size <= 0):
        	return (None, img_sem)
        	
        x_rel = round(min_x / rgb_width, 2)
        y_rel = round(min_y / rgb_height, 2)
        
        return (street_object_image, img_sem, x_rel, y_rel)


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
        lower_red = np.array([0, 50, 50], dtype="uint8")
        upper_red = np.array([15, 255, 255], dtype="uint8")

        # red in the range of purple hue
        lower_violet = np.array([140, 50, 50], dtype="uint8")
        upper_violet = np.array([180, 255, 255], dtype="uint8")

        #red mask
        red_mask_orange = cv2.inRange(frame_hsv, lower_red, upper_red)  
        red_mask_violet = cv2.inRange(frame_hsv, lower_violet, upper_violet) 

        red_mask_full = red_mask_orange + red_mask_violet 

        # green
        lower_green = np.array([35, 85, 110], dtype="uint8")
        upper_green = np.array([70, 255, 255], dtype="uint8")
        green_mask = cv2.inRange(frame_hsv, lower_green, upper_green)


        # yellow
        lower_yellow = np.array([20, 50, 50], dtype="uint8")
        upper_yellow = np.array([30, 255, 255], dtype="uint8")
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


    def publish_detection(self, detections):
        """
        Publish the detections to the relevant topics

        :param detections: Detections as an instance of PerceptionInfo
        :return:
        """

        if not type(detections) is PerceptionInfo or not detections or not detections.objects:
            self.perception_info_publisher.publish(PerceptionInfo())
            self.traffic_light_publisher.publish('')
            return None
            

        # if detection actually has a value and is of type PerceptionInfo
        self.perception_info_publisher.publish(detections)
        
        if detections.objects and type(detections.objects) is list:
            for i in range(len(detections.objects)):
                if detections.objects[i] == "street_signs":
                    speed_limit = float(detections.values[i])
                    self.speed_limit_publisher.publish(speed_limit)
            # red dominates over yellow and green
            if ('red' in detections.values):
            	self.traffic_light_publisher.publish('red')
            # yellow dominates over green
            elif ('yellow' in detections.values):
            	self.traffic_light_publisher.publish('yellow')
            elif ('green' in detections.values):
            	self.traffic_light_publisher.publish('green')
            else:
           		self.traffic_light_publisher.publish('')
        else:
            self.traffic_light_publisher.publish('')
            		

    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.rgb_img is None and not self.semantic_segmentation_img is None:
                street_object_detections = self.detect_street_objects(self.rgb_img, self.semantic_segmentation_img)

                if street_object_detections:
                	self.publish_detection(street_object_detections)
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass


def main():
    rospy.init_node('street_object_detector', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    detector = StreetObjectDetector(role_name)

    try:
        detector.run()
    finally:
        del detector

if __name__ == "__main__":
    main()
