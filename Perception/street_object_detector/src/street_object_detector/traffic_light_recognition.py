import cv2
import numpy as np

def detect_traffic_light_color(img, pixel_count_threshold):
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
