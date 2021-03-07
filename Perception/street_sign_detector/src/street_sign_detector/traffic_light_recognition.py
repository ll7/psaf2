import cv2
import numpy as np

def detect_traffic_light_color(img):

    ## convert to hsv
    frame_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # red represents two regions in the HSV space
    lower_red = np.array([0, 85, 110], dtype="uint8")
    upper_red = np.array([15, 255, 255], dtype="uint8")

    # red in the range of purple hue
    lower_violet = np.array([165, 85, 110], dtype="uint8")
    upper_violet = np.array([180, 255, 255], dtype="uint8")

    #red mask
    red_mask_orange = cv2.inRange(frame_hsv, lower_red, upper_red)  
    red_mask_violet = cv2.inRange(frame_hsv, lower_violet, upper_violet) 

    red_mask_full = red_mask_orange + red_mask_violet 

    # green
    lower_green = np.array([40, 85, 110], dtype="uint8")
    upper_green = np.array([91, 255, 255], dtype="uint8")
    green_mask = cv2.inRange(frame_hsv, lower_green, upper_green)


    # yellow
    lower_yellow = np.array([22, 93, 0], dtype="uint8")
    upper_yellow = np.array([45, 255, 255], dtype="uint8")
    yellow_mask = cv2.inRange(frame_hsv, lower_yellow, upper_yellow)

    output_img = frame_hsv.copy()
    output_img[np.where(red_mask_full == 0) and np.where(green_mask == 0) and np.where(yellow_mask == 0)] = 0
    output_img[np.where(yellow_mask > 0)] = [255, 0, 0]
    output_img[np.where(green_mask > 0)] = [0, 255, 0]
    output_img[np.where(red_mask_full > 0)] = [0, 0, 255]
    #print(np.average(output_img))
    average_pixel = np.average(output_img, axis=(0, 1))

    traffic_light_color = ''
  
    if (average_pixel[0]>average_pixel[1]) and (average_pixel[0]>average_pixel[2]):
        traffic_light_color = 'red'
    elif (average_pixel[2]>average_pixel[0]) and (average_pixel[2]>average_pixel[1]):
        traffic_light_color = 'yellow'
    elif (average_pixel[1]>average_pixel[0]) and (average_pixel[1]>average_pixel[2]):
        traffic_light_color = 'green'
    else:
        #traffic_light_color = 'off' 
        #traffic_light_color = ','.join(str(x) for x in average_pixel)
        average_pixel_off = np.average(output_img, axis=(0, 1))
        traffic_light_color = ','.join(str(x) for x in average_pixel_off)


    return traffic_light_color
    