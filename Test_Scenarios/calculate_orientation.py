import glob
import os
import sys
import time
import csv

from numpy import random

import tf 
# from tf_conversions.transformations import quaternion_from_euler

def main():
    pitch = 0.20
    yaw = -180.0
    roll = 0.0

    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    print(f"yaw = {yaw}")
    print(f"x = {q[0]}")
    print(f"y = {q[1]}")
    print(f"z = {q[2]}")
    print(f"w = {q[3]}")
    

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
