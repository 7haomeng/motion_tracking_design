#!/usr/bin/env python3

import numpy as np
import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

image = cv2.imread('./dataSet/hao/masks/frame0000.png', cv2.IMREAD_COLOR)

for x in range(1, 480):
    for y in range(1, 640): 
        pixel = image[x, y]
        print(pixel)
