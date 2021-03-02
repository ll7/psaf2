import cv2
import numpy as np

def detect_traffic_light_color(img):

    ## convert to hsv
    hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)

    # Define lower and uppper limits of what we call "gray"
    gray_lo = np.array([0, 0, 0])
    gray_hi = np.array([220, 220, 220])
    # Mask image to only select gray
    mask_gray = cv2.inRange(hsv, gray_lo, gray_hi)
    # Change image to red where we found grey
    img[mask_gray > 0] = (0, 0, 0)

    # Define lower and uppper limits of what we call "brown"
    brown_lo = np.array([10, 0, 0])
    brown_hi = np.array([20, 255, 255])
    # Mask image to only select browns
    mask_brown = cv2.inRange(hsv, brown_lo, brown_hi)
    # Change image to red where we found brown
    img[mask_brown > 0] = (0, 0, 0)

    # change everything to black where pixel is white
    white_lo = np.array([0, 0, 168])
    white_hi = np.array([172, 111, 255])
    mask_white = cv2.inRange(hsv, white_lo, white_hi)
    img[mask_white > 0] = (0, 0, 0)

################# find green ################
    rows, cols, _ = img.shape
    color_B = 0  # blue
    color_G = 0  # green
    color_R = 0  # red
    color_N = 0  # neutral/gray color

    for i in range(rows):
        for j in range(cols):
            k = img[i, j]
            if k[0] > k[1] and k[0] > k[2]:
                color_B = color_B + 1
                continue
            if k[1] > k[0] and k[1] > k[2]:
                color_G = color_G + 1
                continue
            if k[2] > k[0] and k[2] > k[1]:
                color_R = color_R + 1
                continue
            color_N = color_N + 1
    pix_total = rows * cols
    b = color_B / pix_total
    g = color_G / pix_total
    r = color_R / pix_total
    n = color_N / pix_total

    avg_color_per_row = np.average(img, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)

    traffic_light_color = ''
    
    if (g > r) & (g > b):
#        print("Traffic Light GREEN")
#        print("Traffic Light GREEN")
        traffic_light_color = 'green'

    elif (r > g) & (r > b) & (r > 0.017) & (n < 0.99):
        #if avg_color[0] > 2 and avg_color[1] < 12 and avg_color[2] > 11:
        if avg_color[0] < 9 and avg_color[1] > 7 and avg_color[1] > 6:
#            print("Traffic Light YELLOW")
#            print("Traffic Light YELLOW")
            traffic_light_color = 'yellow'
        else:
#            print("Traffic Light RED")
#            print("Traffic Light RED")
            traffic_light_color = 'red'

#    else:
#        print("Traffic Light not identified")


    return traffic_light_color
