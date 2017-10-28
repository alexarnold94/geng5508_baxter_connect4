import numpy as np
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to image")
args = vars(ap.parse_args())

image = cv2.imread(args["image"])
image = cv2.resize(image, (1000,800))
image_cp = image.copy()

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_yellow = np.array([20, 60, 60])
upper_yellow = np.array([45, 255, 255])
yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
cv2.imshow('Yellow', yellow)

lower_red = np.array([165, 60, 60])
upper_red = np.array([180, 255, 255])
red = cv2.inRange(hsv, lower_red, upper_red)
cv2.imshow('Red', red)


# Cannot seem to detect red objects in HSV space! Maybe check with Baxter camera

red_circles = cv2.HoughCircles(red, cv2.cv.CV_HOUGH_GRADIENT, 3.5, 50, maxRadius=100)
yellow_circles = cv2.HoughCircles(yellow, cv2.cv.CV_HOUGH_GRADIENT, 3.5, 50, maxRadius=100)

closest_point = np.array((0, 0))
origin = np.array((500,400))
best_dist = 1000000

cv2.circle(image_cp, (origin[0], origin[1]), 3, (0,255,0), 3) # Centre point

if red_circles is not None:
    red_circles = np.round(red_circles[0,:]).astype("int")

    for (x, y, r) in red_circles:
        cv2.circle(image_cp, (x, y), 3, (0, 0, 255), 3) # for debugging

if yellow_circles is not None:
    yellow_circles = np.round(yellow_circles[0,:]).astype("int")

    for (x, y, r) in yellow_circles:
        dist = np.linalg.norm(origin - np.array((x, y)))
        print dist, best_dist
        if dist < best_dist:
            closest_point = [x, y]
            best_dist = dist
        cv2.circle(image_cp, (x, y), 3, (0, 0, 255), 3) # for debugging

# print line to closest game piece
cv2.line(image_cp, (origin[0], origin[1]), (closest_point[0], closest_point[1]), (255,0,255))
cv2.imshow("output", image_cp)
cv2.waitKey(0)
