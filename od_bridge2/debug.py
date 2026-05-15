import json
import os
import sys
import cv2
import numpy as np

if len(sys.argv) < 2:
    print("Usage: python debug.py <image_path>")
    sys.exit(1)

img = cv2.imread(sys.argv[1])
if img is None:
    print(f"Could not read {sys.argv[1]}")
    sys.exit(1)

h, w = img.shape[:2]
scale = min(1.0, 900 / max(h, w))
if scale < 1.0:
    img = cv2.resize(img, (int(w * scale), int(h * scale)))

blurred = cv2.GaussianBlur(img, (5, 5), 0)
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

samples = []

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        h_, s_, v_ = hsv[y, x]
        samples.append((int(h_), int(s_), int(v_)))
        print(f"clicked ({x},{y})  HSV = ({h_}, {s_}, {v_})")
        if len(samples) >= 2:
            arr = np.array(samples)
            lo = np.clip(arr.min(axis=0) - np.array([5, 10, 20]), 0, 255)
            hi = np.clip(arr.max(axis=0) + np.array([5, 30, 20]), 0, 255)
            print(f"  suggested lower = {tuple(int(v) for v in lo)}")
            print(f"  suggested upper = {tuple(int(v) for v in hi)}")

cv2.namedWindow("image")
cv2.setMouseCallback("image", on_mouse)

def nothing(_): pass
cv2.namedWindow("controls")
# Primary hue range
cv2.createTrackbar("H1 lo", "controls",   0, 180, nothing)
cv2.createTrackbar("H1 hi", "controls",  15, 180, nothing)
# Secondary hue range (for red/pink wraparound). Set hi<lo to disable.
cv2.createTrackbar("H2 lo", "controls", 160, 180, nothing)
cv2.createTrackbar("H2 hi", "controls", 180, 180, nothing)
cv2.createTrackbar("S lo", "controls",  40, 255, nothing)
cv2.createTrackbar("S hi", "controls", 140, 255, nothing)
cv2.createTrackbar("V lo", "controls", 120, 255, nothing)
cv2.createTrackbar("V hi", "controls", 235, 255, nothing)
cv2.createTrackbar("kernel", "controls", 9, 51, nothing)
cv2.createTrackbar("min area", "controls", 500, 20000, nothing)
cv2.createTrackbar("circ x100", "controls", 60, 100, nothing)

print("Click pixels on the target to sample HSV. Press 's' to save calibration.json, 'q' to quit.")

CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(sys.argv[1])), "calibration.json")

while True:
    h1_lo = cv2.getTrackbarPos("H1 lo", "controls")
    h1_hi = cv2.getTrackbarPos("H1 hi", "controls")
    h2_lo = cv2.getTrackbarPos("H2 lo", "controls")
    h2_hi = cv2.getTrackbarPos("H2 hi", "controls")
    s_lo = cv2.getTrackbarPos("S lo", "controls")
    s_hi = cv2.getTrackbarPos("S hi", "controls")
    v_lo = cv2.getTrackbarPos("V lo", "controls")
    v_hi = cv2.getTrackbarPos("V hi", "controls")
    k    = max(1, cv2.getTrackbarPos("kernel", "controls") | 1)
    min_area = cv2.getTrackbarPos("min area", "controls")
    circ_thresh = cv2.getTrackbarPos("circ x100", "controls") / 100.0

    m1 = cv2.inRange(hsv, np.array([h1_lo, s_lo, v_lo]),
                          np.array([h1_hi, s_hi, v_hi]))
    if h2_hi >= h2_lo:
        m2 = cv2.inRange(hsv, np.array([h2_lo, s_lo, v_lo]),
                              np.array([h2_hi, s_hi, v_hi]))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = m1
    kernel = np.ones((k, k), np.uint8)
    solid = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    solid = cv2.morphologyEx(solid, cv2.MORPH_OPEN, kernel)

    display = img.copy()
    contours, _ = cv2.findContours(solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area <= best_area:
            continue
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circ = 4 * np.pi * area / (perim * perim)
        if circ < circ_thresh:
            continue
        best, best_area = cnt, area
    if best is not None:
        x, y, w_, h_ = cv2.boundingRect(best)
        cv2.rectangle(display, (x, y), (x + w_, y + h_), (0, 255, 0), 2)

    cv2.imshow("image", display)
    cv2.imshow("mask", solid)
    key = cv2.waitKey(30) & 0xFF
    if key == ord('q'):
        break
    if key == ord('s'):
        calib = {
            "h1_lo": h1_lo, "h1_hi": h1_hi,
            "h2_lo": h2_lo, "h2_hi": h2_hi,
            "s_lo": s_lo, "s_hi": s_hi,
            "v_lo": v_lo, "v_hi": v_hi,
            "kernel": k, "min_area": min_area,
            "circularity": circ_thresh,
        }
        with open(CALIB_PATH, "w") as f:
            json.dump(calib, f, indent=2)
        print(f"saved {CALIB_PATH}")

cv2.destroyAllWindows()
