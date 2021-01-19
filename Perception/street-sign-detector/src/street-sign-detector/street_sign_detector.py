from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from custom_msgs.msg import PerceptionInfo
from time import sleep
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import pathlib


darknet_path = str(pathlib.Path(__file__).parent.parent.absolute() / 'darknet')
sys.path.append(darknet_path)
from darknet_carla import YOLO


class StreetSignDetector:
    """
    Class to use /carla/<role_name>/semantic_segmentation_front/image and /carla/<role_name>/rgb_front/image topic
    to detect street signs and publish them to /psaf/StreetSign topic.
    """

    def __init__(self, role_name):
        self.role_name = role_name
        self.bridge = CvBridge()
        self._semantic_segmentation_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/semantic_segmentation_front/image".format(role_name), Image,
            self.semantic_segmentation_image_updated)

        self._rgb_front_image_subscriber = rospy.Subscriber(
            "/carla/{}/rgb_front/image".format(role_name), Image, self.rgb_image_updated)

        self._control = CarlaEgoVehicleControl()
        self.rgb_img = np.zeros((600, 800, 3), np.uint8)
        self.semantic_segmentation_img = np.zeros((300, 400, 3), np.uint8)
        self.yolo_detector = YOLO()

        # FOR DEBUG ONLY
        self.global_filename_counter = 0

    def get_block_by_pixel(self, img_rgb, img_sem, h_current, w_current, height, width, filename):
        '''
        Finds the street sign in the image of the segmentation camera which includes the pixel at position (h_current,
        w_current), cuts the shape out of the rgb camera image and saves the street sign in a file.

        :param img_rgb: current image of the rgb camera
        :param img_sem: current image of the semantic segmentation camera
        :param h_current: x position of the currently observed pixel
        :param w_current: y position of the currently observed pixel
        :param height: height of img_sem_seg
        :param width: width of img_sem_seg
        :param filename: filename for the next street sign
        :return: the img_sem_seg in which the latest found street sign is removed
        '''
        pixels = []
        current_pixels = []
        pixels.append([h_current, w_current])
        current_pixels.append([h_current, w_current])
        img_sem[h_current, w_current, 2] = 0
        while len(current_pixels) > 0:
            for pixel in current_pixels:
                # check if pixel above is part of the street sign
                if pixel[0] > 0 and img_sem[pixel[0] - 1, pixel[1], 0] == 0 and img_sem[pixel[0] - 1, pixel[1], 1] == 220 and img_sem[pixel[0] - 1, pixel[1], 2] == 220:
                    current_pixels.append([pixel[0] - 1, pixel[1]])
                    pixels.append([pixel[0] - 1, pixel[1]])
                    img_sem[pixel[0] - 1, pixel[1], 0] = 255
                    img_sem[pixel[0] - 1, pixel[1], 1] = 255
                    img_sem[pixel[0] - 1, pixel[1], 2] = 255
                # check if pixel below is part of the street sign
                if pixel[0] < height - 1 and img_sem[pixel[0] + 1, pixel[1], 0] == 0 and img_sem[pixel[0] + 1, pixel[1], 1] == 220 and img_sem[pixel[0] + 1, pixel[1], 2] == 220:
                    current_pixels.append([pixel[0] + 1, pixel[1]])
                    pixels.append([pixel[0] + 1, pixel[1]])
                    img_sem[pixel[0] + 1, pixel[1], 0] = 255
                    img_sem[pixel[0] + 1, pixel[1], 1] = 255
                    img_sem[pixel[0] + 1, pixel[1], 2] = 255
                # check if pixel to the left is part of the street sign
                if pixel[1] > 0 and img_sem[pixel[0], pixel[1] - 1, 0] == 0 and img_sem[pixel[0], pixel[1] - 1, 1] == 220 and img_sem[pixel[0], pixel[1] - 1, 2] == 220:
                    current_pixels.append([pixel[0], pixel[1] - 1])
                    pixels.append([pixel[0], pixel[1] - 1])
                    img_sem[pixel[0], pixel[1] - 1, 0] = 255
                    img_sem[pixel[0], pixel[1] - 1, 1] = 255
                    img_sem[pixel[0], pixel[1] - 1, 2] = 255
                # check if pixel to the right is part of the street sign
                if pixel[1] < width - 1 and img_sem[pixel[0], pixel[1] + 1, 0] == 0 and img_sem[pixel[0], pixel[1] + 1, 1] == 220 and img_sem[pixel[0], pixel[1] + 1, 2] == 220:
                    current_pixels.append([pixel[0], pixel[1] + 1])
                    pixels.append([pixel[0], pixel[1] + 1])
                    img_sem[pixel[0], pixel[1] + 1, 0] = 255
                    img_sem[pixel[0], pixel[1] + 1, 1] = 255
                    img_sem[pixel[0], pixel[1] + 1, 2] = 255
                current_pixels.remove(pixel)

        # ignore too small images
        if(len(pixels) < 50):
            return img_sem

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

        # map every pixel of the semantic segmentation image to a 2x2 part of the rgb image (because the rgb
        # resolution is two times the resolution of the semantic segmentation camera)
        for pixel in pixels:
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2, 0] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2), 0]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2, 0] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2), 0]
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2 + 1, 0] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2 + 1), 0]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2 + 1, 0] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2 + 1), 0]

            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2, 1] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2), 1]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2, 1] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2), 1]
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2 + 1, 1] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2 + 1), 1]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2 + 1, 1] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2 + 1), 1]

            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2, 2] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2), 2]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2, 2] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2), 2]
            street_sign_image[(pixel[0] - min_height) * 2, (pixel[1] - min_width) * 2 + 1, 2] = img_rgb[
                (pixel[0] * 2), (pixel[1] * 2 + 1), 2]
            street_sign_image[(pixel[0] - min_height) * 2 + 1, (pixel[1] - min_width) * 2 + 1, 2] = img_rgb[
                (pixel[0] * 2 + 1), (pixel[1] * 2 + 1), 2]
        rospy.loginfo("New street sign detected")



        self.detection = PerceptionInfo()
        detection, img_detected = self.yolo_detector.detect(street_sign_image)
        cv2.imshow('Demo', img_detected)
        cv2.waitKey(1)

        print(detection)
        # TODO: call image recognition function for 'street_sign_image' here



        # FOR DEBUG ONLY
        cv2.imwrite(str(pathlib.Path(__file__).parent.absolute()) + '/' + filename + '_' + str(self.global_filename_counter) + '.png', street_sign_image)
        self.global_filename_counter += 1

        return img_sem

    def rgb_image_updated(self, data):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def semantic_segmentation_image_updated(self, data):
        try:
            self.semantic_segmentation_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def check_for_street_signs(self, img_rgb, img_sem):
        height, width, channels = img_sem.shape

        filename_counter = 0

        # search for the next pixel of a street sign
        for h in range(height):
            for w in range(width):
                if img_sem[h, w, 0] == 0 and img_sem[h, w, 1] == 220 and img_sem[h, w, 2] == 220:
                    img_sem[h, w, 0] = 255
                    img_sem[h, w, 1] = 255
                    img_sem[h, w, 2] = 255
                    img_sem = self.get_block_by_pixel(img_rgb, img_sem, h, w, height, width, str(filename_counter))
                    filename_counter += 1

    def run(self):
        """
        Control loop
        :return:
        """
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.check_for_street_signs(self.rgb_img, self.semantic_segmentation_img)
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                pass


def main():
    rospy.init_node('vehicle_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    detector = StreetSignDetector(role_name)

    try:
        detector.run()
    finally:
        del detector

if __name__ == "__main__":
    main()
