# Standard Library
import os
import json
from enum import Enum
from typing import Tuple, List
from pathlib import Path
import sys
# External library
from easydict import EasyDict
import cv2
import numpy as np
import wget
import matplotlib.pyplot as plt

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from utils.KE_utils import find_pin_circles

input_path = r"C:\Users\Admin_PC\Desktop\robot\farino\input\only_pins.jpg"
#if not os.path.isfile(input_path):
    #wget.download(img_url, input_path)
opts = EasyDict({
    "x_scale": 1,
    "y_scale": 1
})
result = find_pin_circles(image_path=input_path, opts=opts)


import cv2

image = cv2.imread(input_path)
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
cv2.imwrite(r"C:\Users\Admin_PC\Desktop\robot\farino\output\Gray_marker.jpg", gray)
for circle in result:
    point = circle.get("point")
    radius = circle.get("radius")

    if point is None or radius is None:
        print(f"⚠️ Skipping invalid entry: {circle}")
        continue

    # Access x and y from the Point object
    x, y = int(point.x), int(point.y)
    r = int(radius)

    # Draw the outer circle (green)
    cv2.circle(image, (x, y), r, (0, 255, 0), 2)
    # Draw the center point (red)
    cv2.circle(image, (x, y), 3, (0, 0, 255), -1)
    # Optional: label each circle
    cv2.putText(image, f"({x},{y}) r={r}", (x + 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
cv2.imwrite(r"C:\Users\Admin_PC\Desktop\robot\farino\output\pins_visualized.jpg", image)
cv2.imshow("Detected Circles", image)
cv2.waitKey(0)
cv2.destroyAllWindows()