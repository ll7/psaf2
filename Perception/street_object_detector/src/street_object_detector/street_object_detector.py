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
        self.minimal_object_size_for_detection = 20
        
        # OCR settings
        self.ocr_tesseract_configs = [ "--psm 9 --oem 1", "--psm 13 --oem 1" ]
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
        self.traffic_light_pixel_count_threshold = 5

        # COLOR SETTINGS (B, R, G) (have to be numpy arrays!)
        self.semantic_street_signs_color = np.array([ 0, 220, 220 ], np.uint8)
        self.semantic_traffic_light_color = np.array([ 30, 170, 250 ], np.uint8)
        self.semantic_replace_by_color = np.array([ 255, 255, 255 ], np.uint8)      # sign's color to be (temporarily) replaced by
        
        # There are dead areas in the camera's field of view where street signs and traffic lights can be ignored
        # OFFSET: [top, right, bottom, left]
        self.semantic_street_signs_offset = [0, 0, 0, 0.5]
        self.semantic_traffic_light_offset = [0, 0.35, 0.7, 0.35]
        
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
            "/carla/{}/camera/semantic_segmentation/front/image_segmentation".format(role_name), Image, self.semantic_segmentation_image_updated)

        self._rgb_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/front/image_color".format(role_name), Image, self.rgb_image_updated)

        # PUBLISHERS
        self.speed_limit_publisher = rospy.Publisher(
            "/psaf/{}/speed_limit".format(self.role_name), Float64, queue_size=1, latch=True)

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


    def resize_semantic_segmentation_image(self, img_sem, img_rgb):
        """
        Scales semantic segmentation image to the dimensions of the RGB image in case they are not identically to get a correct mapping
        :param img_sem: image of the semantic segmentation camera
        :param img_rgb: image of the RGB camera
        :return: scaled semantic segmentation image
        """
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
            
        #height_resized, width_resized, channels_resized = img_resized.shape
        #new_img_sem = np.zeros((height_rgb, width_rgb, 3), np.uint8)
        #x_offset = int((width_rgb - width_resized) / 2)
        #y_offset = int((height_rgb - height_resized) / 2)
        
		    # put black bars on the top and the bottom of the semantic segmentation image (if ratio_width < ratio_height)
		    # or on the left and right side (if ratio_width < ratio_height)
        #new_img_sem[y_offset:y_offset + height_resized, x_offset:x_offset + width_resized] = img_resized
        
        #return new_img_sem
        return img_resized
        

    def detect_street_objects(self, img_rgb, img_sem):
        """
        Searches for street objects in the semantic segmentation (img_sem) if present, calls tesseract
        to read the speed limit of steet signs or the traffic light detection for traffic lights

        :param img_rgb: The RGB camera image
        :param img_sem: The semantic segmentation camera image
        :return: If possible, the detection as an instance of PerceptionInfo; None otherwise
        """

        img_sem = self.resize_semantic_segmentation_image(img_sem, img_rgb)
        cv2.imshow("sem", img_sem)
        cv2.waitKey(1)
        height, width, channels = img_sem.shape
            
        detections = PerceptionInfo()
        detections.objects = []
        detections.values = []
        
        stop_img = np.zeros((height, width, channels), np.uint8)
        
        rgb_height = img_rgb.shape[0]
        border_height = int((rgb_height - height) / 2)
        
        street_objects = {}
        
        # iterate through the defined colors to search for
        for color_key in self.semantic_segment_search_colors.keys():
            color = self.semantic_segment_search_colors[color_key]
            offset = self.semantic_segment_search_offsets[color_key]
            street_objects[color_key] = { "count": 0, "objects": [] }
            
            for h in range(int(offset[0] * height), height - int(offset[2] * height)):
                for w in range(int(offset[3] * width), width - int(offset[1] * width)):
                    try:
                        if np.array_equal(img_sem[h, w], color):
                            (street_object_image, img_sem) = self.get_block_by_pixel(img_rgb, img_sem, color, h, w, border_height)
                            
                            # valid result?
                            if not street_object_image is None:
                                cv2.imshow("sign", street_object_image)
                                cv2.waitKey(1)
                                street_objects[color_key]["count"] += 1
                                street_objects[color_key]["objects"].append([ round(w / width, 2), round((h + border_height) / rgb_height, 2), street_object_image.copy() ])
                                    
                    except Exception as e:
                        print_exc()
                        pass
                        
            # if there have been found any pixel blocks of interest, return its detection
            if street_objects[color_key]["count"] > 0:
                if color_key == "street_signs":
                    for (x_rel, y_rel, street_sign_rgb) in street_objects[color_key]["objects"]:
                        ocr_texts = []
                        
                        for tesseract_config in self.ocr_tesseract_configs:
                            ocr_texts.append(self.get_ocr_text(street_sign_rgb, tesseract_config, self.ocr_preprocess_tasks, self.ocr_resize_to))
                    
                        recognized_speed_limit = None
                        for ocr_text in ocr_texts:
                            recognized_number = ''.join([ i for i in ocr_text.split() if i.isdigit() ])
                            
                            if recognized_number in self.speed_limit_values and (recognized_speed_limit is None or int(recognized_number) < int(recognized_speed_limit)):
                                recognized_speed_limit = recognized_number
                                
                            
                        if recognized_speed_limit:
                            detections.objects.append(color_key)
                            detections.values.append(recognized_speed_limit)
                            detections.relative_x_coord.append(x_rel)
                            detections.relative_y_coord.append(y_rel)
            
                if color_key == "traffic_lights":
                    for (x_rel, y_rel, traffic_light_rgb) in street_objects[color_key]["objects"]:
                        traffic_light_color = detect_traffic_light_color(traffic_light_rgb, self.traffic_light_pixel_count_threshold)
                        
                        if traffic_light_color != '':
                            detections.objects.append(color_key)
                            detections.values.append(traffic_light_color)
                            detections.relative_x_coord.append(x_rel)
                            detections.relative_y_coord.append(y_rel)

        return detections
        
    def get_ocr_text(self, street_sign_rgb, tesseract_config, preprocess_tasks, resize_to):
        gray = cv2.cvtColor(street_sign_rgb, cv2.COLOR_BGR2GRAY)
        
        if resize_to != 0:
            resize_factor = resize_to / gray.shape[1]
            new_width = int(gray.shape[1] * resize_factor)
            new_height = int(gray.shape[0] * resize_factor)
            gray = cv2.resize(gray, (new_width, new_height))

        if "b" in preprocess_tasks:
        	gray = cv2.medianBlur(gray, 3)
        if "t" in preprocess_tasks:
        	gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 1)
        	
        cv2.imshow("ocr", gray)
        cv2.waitKey(1)
        	
        text = pytesseract.image_to_string(gray, config=tesseract_config, lang='eng')
        text = text.strip()
        
        for replacement in self.speed_limit_replacements:
            text = text.replace(replacement[0], replacement[1])
        
        return text
        
        
    def get_block_by_pixel(self, img_rgb, img_sem, search_color, h_current, w_current, border_height):
        """
        Finds the object in the image of the segmentation camera with the given color and which includes the pixel at position (h_current,
        w_current), cuts the shape out of the rgb camera image and returns the cut out object

        :param img_rgb: current image of the RGB camera
        :param img_sem: current image of the semantic segmentation camera
        :param search_color: the color searched for (BRG!), e.g. [ 0, 220, 220 ]
        :param h_current: x position of the currently observed pixel
        :param w_current: y position of the currently observed pixel
        :param border_height: offset between img_rgb and img_sem (add this value to the img_rgb y position)
        :return: returns a list of the extracted img_sem part in which the latest found object is contained and of the img_sem with
        all the pixels of the current object block converted to semantic_replace_by_color - or it returns None for the extracted
        img_sem part instead, when the block of pixels belonging together is too small and below the configured threshold (e.g. the
        object is too far away and therefore too small until now)
        """
            
        pixels = []
        current_pixels = []
        pixels.append([h_current, w_current])
        current_pixels.append([h_current, w_current])
        (height, width, dimensions) = img_sem.shape
        
        img_sem[h_current, w_current] = self.semantic_replace_by_color

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

        # ignore too small images, return None for the extracted img_sem part
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

        # create new image for the detected street object
        street_object_image = np.zeros((new_height+1, new_width+1, 3), np.uint8)
        street_object_image[:, :] = (255, 255, 255)

        # map every pixel of the semantic segmentation image to a 2x2 part of the rgb image (because the rgb
        # resolution is two times the resolution of the semantic segmentation camera)
        for pixel in pixels:
            rgb_pixel_y = pixel[0] + border_height
            street_object_image[(pixel[0] - min_height), (pixel[1] - min_width)] = img_rgb[
                rgb_pixel_y, (pixel[1])]

            street_object_image[(pixel[0] - min_height), (pixel[1] - min_width)] = img_rgb[
                rgb_pixel_y, (pixel[1])]

            street_object_image[(pixel[0] - min_height), (pixel[1] - min_width)] = img_rgb[
                rgb_pixel_y, (pixel[1])]

            street_object_image[(pixel[0] - min_height), (pixel[1] - min_width)] = img_rgb[
                rgb_pixel_y, (pixel[1])]

        return (street_object_image, img_sem)


    def publish_detection(self, detections):
        """
        Publish the detections to the relevant topics

        :param detections: Detections as an instance of PerceptionInfo
        :return:
        """

        if not type(detections) is PerceptionInfo or not detections or not detections.objects:
            self.perception_info_publisher.publish(PerceptionInfo())
            return None
            

        # if detection actually has a value and is of type PerceptionInfo
        self.perception_info_publisher.publish(detections)
        
        if detections.objects and type(detections.objects) is list:
            for i in range(len(detections.objects)):
                if detections.objects[i] == "street_signs":
                    speed_limit = float(detections.values[i])
                    self.speed_limit_publisher.publish(speed_limit)


    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(2)
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
