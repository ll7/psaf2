from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import traceback
import sys
import argparse
import pathlib

from PIL import Image

darknet_path = str(pathlib.Path(__file__).parent.parent.parent.absolute() / 'darknet')
sys.path.append(darknet_path)
import darknet
from pathlib import Path

def convertBack(x, y, w, h):
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return xmin, ymin, xmax, ymax


def cvDrawBoxes(detections, img):
    for label, confidence, bbox in detections:
        x, y, w, h = (bbox[0], bbox[1], bbox[2], bbox[3])
        name_tag = label
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h))
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)
        cv2.rectangle(img, pt1, pt2, (0, 255, 0), 1)

        try:
            cv2.putText(img,
                        label +
                        " [" + str(round(float(confidence) * 100, 2)) + "]",
                        (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        [0, 255, 0], 2)

        except Exception as e:
            print(label," / ", confidence)
            traceback.print_exc()
            pass
            
    return img


class YOLO(object):
    def __init__(self):
        super(YOLO, self).__init__()
        self.netMain = None
        self.metaMain = None
        self.altNames = None

        parentPath = Path(__file__).parent.parent.parent.absolute() # i.e. street_sign_detector package root
        configPath = os.path.join(parentPath, "yolo-obj.cfg")
        weightPath = os.path.join(parentPath, "yolo-obj_last.weights")
        metaPath = os.path.join(parentPath, "data/obj.data")
        
        if not os.path.exists(configPath):
            raise ValueError("Invalid config path `" +
                            os.path.abspath(configPath)+"`")
        if not os.path.exists(weightPath):
            raise ValueError("Invalid weight path `" +
                            os.path.abspath(weightPath)+"`")
        if not os.path.exists(metaPath):
            raise ValueError("Invalid data file path `" +
                            os.path.abspath(metaPath)+"`")

        self.network, self.class_names, self.class_colors = darknet.load_network(configPath,  metaPath, weightPath, batch_size=1)

        # Create an image we reuse for each detect
        self.darknet_image = darknet.make_image(darknet.network_width(self.network),
                                        darknet.network_height(self.network), 3)


    def detect(self, input_image=None):
        if type(input_image) is str:
            image = np.array(Image.open(input_image))
            frame_read = image
        else:
            frame_read = input_image


        frame_rgb = cv2.cvtColor(frame_read, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb,
                                    (darknet.network_width(self.network),
                                    darknet.network_height(self.network)),
                                    interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(self.darknet_image,frame_resized.tobytes())
        
        detections = darknet.detect_image(self.network,
                                          self.class_names,
                                          self.darknet_image,
                                          thresh=0.1)
                
        image = cvDrawBoxes(detections, frame_resized)
        
        return detections, image
   

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Object recognition')
    parser.add_argument("-src", "--src",
                        dest="src",
                        type=str,
                        required=True,
                        help='your image path')
    args = parser.parse_args()
    
    detector = YOLO()
    pred = detector.detect(args.src)
    print(pred[0])
