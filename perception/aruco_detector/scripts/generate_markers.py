#!/usr/bin/env python3
"""
Generate the first 10 markers of the 4×4_50 dictionary as PNGs.
"""

import cv2 as cv
from cv2 import aruco
import pathlib

out_dir = pathlib.Path("markers")
out_dir.mkdir(exist_ok=True)

dict4x4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
size_px = 400

for mid in range(10):
    img = aruco.generateImageMarker(dict4x4, mid, size_px)
    cv.imwrite(str(out_dir / f"marker_{mid}.png"), img)
print("Markers written to ./markers/")
