import time

import cv2
import numpy as np

LOW = (100, 120, 60)
HIGH = (130, 255, 255)
MIN_AREA = 150
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

frame = 0


def score(contour):
    area = cv2.contourArea(contour)
    if area < MIN_AREA:
        return -1.0
    hull = cv2.contourArea(cv2.convexHull(contour))
    solidity = area / hull if hull > 0 else 0.0
    return area * solidity


def runPipeline(image, llrobot):
    global frame
    start = time.perf_counter()
    frame += 1
    low, high = LOW, HIGH
    debug = False
    if llrobot is not None and len(llrobot) >= 7 and llrobot[6] == 1:
        low = tuple(int(v) for v in llrobot[0:3])
        high = tuple(int(v) for v in llrobot[3:6])
    if llrobot is not None and len(llrobot) >= 8:
        debug = llrobot[7] == 1
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = np.array([[]])
    out = [0, frame, 0, 0, 0, len(contours), 0]
    scored = [(score(c), c) for c in contours]
    scored = [s for s in scored if s[0] > 0]
    if scored:
        best = max(scored, key=lambda s: s[0])[1]
        m = cv2.moments(best)
        if m["m00"] > 0:
            cx = m["m10"] / m["m00"]
            cy = m["m01"] / m["m00"]
            out[0:5] = [1, frame, cx, cy, m["m00"]]
            x, y, w, h = cv2.boundingRect(best)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.circle(image, (int(cx), int(cy)), 4, (0, 0, 255), -1)
    if debug:
        cv2.drawContours(image, contours, -1, (255, 0, 255), 1)
    out[6] = (time.perf_counter() - start) * 1000.0
    return best, image, out
