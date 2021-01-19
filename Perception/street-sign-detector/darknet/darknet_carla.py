from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import darknet

from PIL import Image
import sys
import argparse
from pathlib import Path

def convertBack(x, y, w, h):
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return xmin, ymin, xmax, ymax


def cvDrawBoxes(detections, img):
    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h))
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)
        cv2.rectangle(img, pt1, pt2, (0, 255, 0), 1)
        cv2.putText(img,
                    detection[0].decode() +
                    " [" + str(round(detection[1] * 100, 2)) + "]",
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    [0, 255, 0], 2)
    return img


class YOLO(object):
    def __init__(self):
        super(YOLO, self).__init__()
        self.netMain = None
        self.metaMain = None
        self.altNames = None

        parentPath = Path(__file__).parent.absolute()
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
        if self.netMain is None:
            self.netMain = darknet.load_net_custom(configPath.encode(
                "ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1
        if self.metaMain is None:
            self.metaMain = darknet.load_meta(metaPath.encode("ascii"))
        if self.altNames is None:
            try:
                with open(metaPath) as metaFH:
                    metaContents = metaFH.read()
                    import re
                    match = re.search("names *= *(.*)$", metaContents,
                                    re.IGNORECASE | re.MULTILINE)
                    if match:
                        result = match.group(1)
                    else:
                        result = None
                    try:
                        if os.path.exists(result):
                            with open(result) as namesFH:
                                namesList = namesFH.read().strip().split("\n")
                                self.altNames = [x.strip() for x in namesList]
                    except TypeError:
                        pass
            except Exception:
                pass

        # Create an image we reuse for each detect
        self.darknet_image = darknet.make_image(darknet.network_width(self.netMain),
                                        darknet.network_height(self.netMain),3)

    def detect(self, input_image=None):
        if type(input_image) is str:
            image = np.array(Image.open(input_image))
            frame_read = image
        else:
            frame_read = input_image


        frame_rgb = cv2.cvtColor(frame_read, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb,
                                    (darknet.network_width(self.netMain),
                                    darknet.network_height(self.netMain)),
                                    interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(self.darknet_image,frame_resized.tobytes())
        
        detections = darknet.detect_image(self.netMain,
                                          self.metaMain,
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