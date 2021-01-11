from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from time import sleep
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


class ScriptedControl:

    def __init__(self, role_name):
        self.role_name = role_name
        self._target_speed_subscriber = rospy.Subscriber(
            "/carla/{}/semantic_segmentation/image".format(role_name), Image, self.semantic_segmentation_image_updated)

        self._target_speed_subscriber = rospy.Subscriber(
            "/carla/{}/rgb/image".format(role_name), Image, self.rgb_updated)

        self._control = CarlaEgoVehicleControl()
        self.rgb_img = np.zeros((500, 500, 3), np.uint8)

    def get_block_by_pixel(self, img_sem_seg, h_current, w_current, height, width, filename):
        '''
        Finds the street sign in the image of the segmentation camera which includes the pixel at position (h_current,
        w_current), cuts the shape out of the rgb camera image and saves the street sign in a file.

        :param img_sem_seg: current image of the semantic segmentation camera
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
        img_sem_seg[h_current, w_current, 2] = 0
        while len(current_pixels) > 0:
            for pixel in current_pixels:
                # pixel above
                if pixel[0] > 0 and img_sem_seg[pixel[0] - 1, pixel[1], 0] == 0 and img_sem_seg[pixel[0] - 1, pixel[1], 1] == 0 and img_sem_seg[
                    pixel[0] - 1, pixel[1], 2] == 255:
                    current_pixels.append([pixel[0] - 1, pixel[1]])
                    pixels.append([pixel[0] - 1, pixel[1]])
                    img_sem_seg[pixel[0] - 1, pixel[1], 2] = 0
                # pixel below
                if pixel[0] < height - 1 and img_sem_seg[pixel[0] + 1, pixel[1], 0] == 0 and img_sem_seg[
                    pixel[0] + 1, pixel[1], 1] == 0 and img_sem_seg[pixel[0] + 1, pixel[1], 2] == 255:
                    current_pixels.append([pixel[0] + 1, pixel[1]])
                    pixels.append([pixel[0] + 1, pixel[1]])
                    img_sem_seg[pixel[0] + 1, pixel[1], 2] = 0
                # pixel left
                if pixel[1] > 0 and img_sem_seg[pixel[0], pixel[1] - 1, 0] == 0 and img_sem_seg[pixel[0], pixel[1] - 1, 1] == 0 and img_sem_seg[
                    pixel[0], pixel[1] - 1, 2] == 255:
                    current_pixels.append([pixel[0], pixel[1] - 1])
                    pixels.append([pixel[0], pixel[1] - 1])
                    img_sem_seg[pixel[0], pixel[1] - 1, 2] = 0
                # pixel right
                if pixel[1] < width - 1 and img_sem_seg[pixel[0], pixel[1] + 1, 0] == 0 and img_sem_seg[
                    pixel[0], pixel[1] + 1, 1] == 0 and img_sem_seg[pixel[0], pixel[1] + 1, 2] == 255:
                    current_pixels.append([pixel[0], pixel[1] + 1])
                    pixels.append([pixel[0], pixel[1] + 1])
                    img_sem_seg[pixel[0], pixel[1] + 1, 2] = 0
                current_pixels.remove(pixel)

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
        blank_image = np.zeros((new_height, new_width, 3), np.uint8)
        blank_image[:, :] = (255, 255, 255)
        for pixel in pixels:
            blank_image[pixel[0] - min_height - 1, pixel[1] - min_width - 1, 0] = self.semantic_segmentation_img[pixel[0] - min_height - 1, pixel[1] - min_width - 1, 0]
            blank_image[pixel[0] - min_height - 1, pixel[1] - min_width - 1, 1] = self.semantic_segmentation_img[pixel[0] - min_height - 1, pixel[1] - min_width - 1, 1]
            blank_image[pixel[0] - min_height - 1, pixel[1] - min_width - 1, 2] = self.semantic_segmentation_img[pixel[0] - min_height - 1, pixel[1] - min_width - 1, 2]
        cv2.imwrite(filename + '.png', blank_image)
        return img_sem_seg

    def rgb_image_updated(self, data):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def semantic_segmentation_image_updated(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = img.shape
        filename_counter = 0

        for h in range(height):
            for w in range(width):
                if img[h, w, 0] == 0 and img[h, w, 1] == 0 and img[h, w, 2] == 255:
                    img[h, w, 0] = 255
                    img[h, w, 1] = 255
                    img = self.get_block_by_pixel(img, h, w, height, width, str(filename_counter))
                    filename_counter += 1
    def run(self):
        while True:
            sleep(0.2)


def main():
    rospy.init_node('carla_manual_control', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")

    c = ScriptedControl(role_name)
    c.run()


if __name__ == '__main__':
    main()