#!/usr/bin/python

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import numpy as np
import threading
import math

from copy import deepcopy
from os import system

import rospy

import cv2

#from cv2 import cv
from cv_bridge import CvBridge

from geometry_msgs.msg import (
  PolygonStamped,
)

from sensor_msgs.msg import (
  Image,
)

from std_msgs.msg import (
  String,
)

import baxter_interface


class ConnectFourVision(object):
    def __init__(self, limb):
        # Start camera
        self._side = limb
        self._camera_name = self._side + "_hand_camera"
        print ("Opening " + self._camera_name + "...")
        try:
            self._head_camera = baxter_interface.CameraController("head_camera")
            print ("Attempting to turn off the head camera...")
            self._head_camera.close()
        except Exception:
            pass
        self._camera = baxter_interface.CameraController(self._camera_name)
        self._camera.open()
        self._camera.resolution = [1280, 800]
        # HAL9001: Added camera origin and adjusted gain
        self._origin = np.array((710,320))
        self._camera.gain = 20
        self._game_piece_diameter = 0.015

        self.grid = [[0 for _i in range(7)] for _j in range(6)]
        self.cv_image = None
        self._bridge = CvBridge()

        self.yellow_sample = ()
        self.red_sample = ()
        self.blue_sample = ()

        self.yellows = None
        self.reds = None
        self.blues = None

        self._roi_points = [[100, 100], [200, 100], [200, 200], [100, 200]]
        self._roi_move = False
        self._point_selected = -1
        self._gain_slider = 20
        self._red_thresh = 100
        self._yellow_thresh = 100
        self._slider_time = rospy.Time.now()
        self._gain_set = False
        self._text = ['X', 'Y', 'R', 'G', 'B']
        self._pixel = dict()
        for label in self._text:
            self._pixel[label] = 0.0
        self._vector = dict()
        self._grid = [[0 for _i in range(7)] for _j in range(6)]
        self._pnts = [[0 for i in range(8)] for j in range(7)]
        self._user_cnt = 0
        self._baxter_cnt = 0

        # initialize images
        self._np_image = np.zeros((300, 300, 3), np.uint8)
        self._image_grid = np.zeros((300, 300, 3), np.uint8)
        self._yellow = np.zeros((300, 300), np.uint8)
        self._red = np.zeros((300, 300), np.uint8)
        self._projected = np.zeros((300, 300, 3), np.uint8)

        self.subLock = threading.Lock()

        camera_topic = '/cameras/' + self._camera_name + '/image'
        _camera_sub = rospy.Subscriber(
            camera_topic,
            Image,
            self._on_camera)

        roi_topic = '/connect_four/localize/grid_pixels'
        _roi_sub = rospy.Subscriber(
            roi_topic,
            PolygonStamped,
            self._on_roi)

        board_state_topic = '/vision/connect_four_state'
        self._board_state_pub = rospy.Publisher(
            board_state_topic,
            String, queue_size=10)

        # HAL9001: Added topic publisher for updating the game piece location
        nearest_piece_topic = 'vision/connect_four_piece'
        self._nearest_piece_pub = rospy.Publisher(
            nearest_piece_topic,
            String, queue_size=10)

        print 'All set! Starting to process images!'
        self._process_images()

    def _show_image(self):
        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()

        for idx, points in enumerate(self._roi_points):
            cv2.circle(local_image, (points[0], points[1]), 5, (255, 0, 0), 2)

        cv2.polylines(local_image, np.int32([np.array(self._roi_points)]),
                      1, (0, 255, 0), 2)

        cv2.imshow("Connect Four RGB", local_image)

        cv2.setMouseCallback("Connect Four RGB", self._on_mouse_click, 0)
        cv2.createTrackbar("Gain", "Connect Four RGB", self._gain_slider,
                          100, self._on_gain_slider)
        cv2.createTrackbar("Red Threshold", "Connect Four RGB",
                          self._red_thresh, 500, self._on_red_slider)
        cv2.createTrackbar("Yellow Threshold", "Connect Four RGB",
                          self._yellow_thresh, 500, self._on_yellow_slider)
        cv2.waitKey(3)

    def _process_images(self):
        while not rospy.is_shutdown():
            # gain changed from slider settled
            if (rospy.Time.now() - self._slider_time > rospy.Duration(3.0)
                and self._gain_set == True):
                self._gain_set = False
                print 'Setting GAIN!'
                self._camera.gain = self._gain_slider
            # process red/yellow image
            self._show_image()
            # HAL9001: Call board detection
            self._detect_board()
            self._project_roi()
            self._filter_yellow()
            self._filter_red()
            self._process_colors(deepcopy(self._red), deepcopy(self._yellow))
            self._update_image_grid()

            # publish state
            self._pub_state()

            # HAL9001: Find nearest game piece
            self._nearest_piece()
            rospy.sleep(0.1)

    # HAL9001: Get the relative (x,y) coordinate in Baxter space of the nearest game piece
    def _nearest_piece(self):
        # Get local copy of camera image
        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()

        # Convert the image to HSV space and
        hsv = cv2.cvtColor(local_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([155, 100, 100])
        upper_red = np.array([190, 255, 255])
        red = cv2.inRange(hsv, lower_red, upper_red)
        # cv2.imshow('Red', red)

        lower_yellow = np.array([10, 40, 40])
        upper_yellow = np.array([55, 255, 255])
        yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # cv2.imshow('Yellow', yellow)

        # Detect game pieces using OpenCV's Hough Circles
        red_circles = cv2.HoughCircles(red, cv2.HOUGH_GRADIENT, 5, 100, minRadius=20, maxRadius=40)
        yellow_circles = cv2.HoughCircles(yellow, cv2.HOUGH_GRADIENT, 5, 100, minRadius=20, maxRadius=40)

        closest_point = np.array((0, 0, 0))
        best_dist = 1000000
        # Mark Baxter's centre point with a green dot
        cv2.circle(local_image, (self._origin[0], self._origin[1]), 3, (0,255,0), 3) # Centre point

        # Calculate the closest red circle
        if red_circles is not None:
            red_circles = np.round(red_circles[0,:]).astype("int")

            for (x, y, r) in red_circles:
                dist = np.linalg.norm(self._origin - np.array((x, y)))
                if dist < best_dist:
                    closest_point = [x, y, r]
                    best_dist = dist
                cv2.circle(local_image, (x, y), r, (0, 255, 255), 3) # for visualisation
                cv2.circle(local_image, (x, y), 2, (0, 255, 255), 2)

        # Calculate the closest yellow circle
        if yellow_circles is not None:
            yellow_circles = np.round(yellow_circles[0,:]).astype("int")

            for (x, y, r) in yellow_circles:
                dist = np.linalg.norm(self._origin - np.array((x, y)))
                if dist < best_dist:
                    closest_point = [x, y, r]
                    best_dist = dist
                cv2.circle(local_image, (x, y), r, (0, 0, 255), 3) # for debugging
                cv2.circle(local_image, (x, y), 2, (0, 0, 255), 2)

        # Show nearest game piece
        cv2.line(local_image, (self._origin[0], self._origin[1]), (closest_point[0], closest_point[1]), (255,0,255), 2)
        # Display image
        cv2.imshow('Nearest game piece', local_image)

        ratio = 0
        if closest_point[0] == 0:
            ratio = 0
        else:
            ratio = self._game_piece_diameter / closest_point[2]
        state = dict()
        state['x'] = ratio * (closest_point[0] - self._origin[0])
        state['y'] = ratio * (closest_point[1] - self._origin[1])
        self._nearest_piece_pub.publish(str(state))

    # HAL9001: For an image of the board, detect the four corners for the ROI
    def _detect_board(self):
        self.subLock.acquire(True)
        image = deepcopy(self._np_image)
        self.subLock.release()

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert to hsv
        lower_blue = np.array([110,60,60]) # blue thresholds
        upper_blue = np.array([130,255,255])
        _blue = cv2.inRange(hsv, lower_blue, upper_blue) # binary for just blue

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25,25)) # kernel for close operator
        closed = cv2.morphologyEx(_blue, cv2.MORPH_CLOSE, kernel) # execute close operator
        kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (40,40)) # kernel for open operator
        opened =  cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel2) # execute open operator

        _, contours, hierarchy = cv2.findContours(opened, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        i = len(contours)
        if i != 0:
            num = 0
            boardArea = 0
            boardIndex = 0

            while (num < i):
                area = cv2.contourArea(contours[num])
                if area > boardArea: #picks contour with largest area as board
                    boardArea = area
                    boardIndex = num
                num = num + 1

            boardContour = contours[boardIndex] #extract just board contour
            j = len(boardContour) #number of contour points
            num= 0
            topLeft = [1280, 800] #define as extreme values that will definitely be overwritten
            topRight = [0, 800]
            bottomLeft = [1280, 0]
            bottomRight = [0, 0]

            while (num < j): #iterate for each coord in contour
                point = boardContour[num]
                point = point[0] #extract list
                pointX = point[0]
                #print(pointX)
                pointY = point[1]
                #print(pointY)

                if (pointX > topRight[0] - 2 or pointX > topRight[0] + 2): #top right coord x> y<
                    if (pointY < topRight[1] - 2 or pointY < topRight[1] + 2):
                        topRight[0] = pointX
                        topRight[1] = pointY

                if (pointX < topLeft[0] - 2 or pointX < topLeft[0] + 2): #top left coord x< y<
                    if (pointY < topLeft[1] - 2 or pointY < topLeft[1] + 2):
                        topLeft[0] = pointX
                        topLeft[1] = pointY

                if (pointX < bottomLeft[0] - 2 or pointX < bottomLeft[0] + 2): #bottom left coord x< y>
                    if (pointY > bottomLeft[1] - 2 or pointY > bottomLeft[1] + 2):
                        bottomLeft[0] = pointX
                        bottomLeft[1] = pointY

                if (pointX > bottomRight[0] - 2 or pointX > bottomRight[0] + 2): #bottom right coord x> y>
                    if (pointY > bottomRight[1] - 2 or pointY > bottomRight[1] + 2):
                        bottomRight[0] = pointX
                        bottomRight[1] = pointY

                num = num + 1

                self._roi_points = [[topLeft[0],topLeft[1]], [topRight[0],topRight[1]], [bottomRight[0],bottomRight[1]], [bottomLeft[0],bottomLeft[1]]]

    def _process_colors(self, red, yellow):
        # look down each column building up from bottom
        self._grid = [[0 for _i in range(7)] for _j in range(6)]
        self._image_grid = deepcopy(self._projected)
        self._user_cnt = 0
        self._baxter_cnt = 0
        for col in xrange(7):
            cur_row = True
            x_offset = 42 * col
            # Look from the bottom up checking if piece is there
            for row in xrange(5, -1, -1):
                if cur_row == True:
                    y_offset = 50 * row
                    red_cnt = 0
                    yellow_cnt = 0
                    # look though each pixel in current grid location
                    if len(yellow) != 300 or len(red) != 300:
                        print 'BAILING - IMAGE SIZE IS UNEXPECTED'
                        return

                    for y in xrange(50):
                        for x in xrange(42):
                            if yellow[y + y_offset, x + x_offset] == 255:
                                yellow_cnt += 1
                            elif red[y + y_offset, x + x_offset] == 255:
                                red_cnt += 1

                    if yellow_cnt > self._yellow_thresh:
                        cv2.putText(self._image_grid,
                                    '2',
                                    (x_offset + 15, y_offset + 30),
                                    cv2.FONT_HERSHEY_COMPLEX_SMALL,
									1, (0, 0, 0)
                        )
                        self._grid[row][col] = 2
                        self._user_cnt += 1
                    elif red_cnt > self._red_thresh:
                        cv2.putText(self._image_grid,
                                    '1',
                                    (x_offset + 15, y_offset + 30),
                                    cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                    1, (0, 0, 0)
                        )
                        self._grid[row][col] = 1
                        self._baxter_cnt += 1
                    else:
                        cur_row = False

    def _update_image_grid(self):
        for idx in xrange(1, 6):
            cv2.line(self._image_grid, (42 * idx, 0), (42 * idx, 300),
                     (0, 255, 0), 1)
            cv2.line(self._image_grid, (0, 50 * idx), (300, 50 * idx),
                     (0, 255, 0), 1)
            cv2.line(self._image_grid, (42 * 6, 0), (42 * 6, 300),
                     (0, 255, 0), 1)
            cv2.imshow('Board State', self._image_grid)

    def _project_roi(self):
        warped_in = np.float32([np.array(self._roi_points)])
        project_out = np.float32([[0, 0], [300, 0], [300, 300], [0, 300]])
        M = cv2.getPerspectiveTransform(warped_in, project_out)
        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()
        self._projected = cv2.warpPerspective(local_image, M, (300, 300))

    def _filter_yellow(self):
        # Finds yellow colors in HSV space
        hsv = cv2.cvtColor(self._projected, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 60, 60])
        upper_yellow = np.array([45, 255, 255])
        self._yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cv2.imshow('Yellow', self._yellow)

    def _filter_red(self):
        # Finds red colors in HSV space
        hsv = cv2.cvtColor(self._projected, cv2.COLOR_BGR2HSV)
        lower_red = np.array([165, 60, 60])
        upper_red = np.array([180, 255, 255])
        self._red = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow('Red', self._red)

    def _pub_state(self):
        state = dict()
        state['baxter_count'] = self._baxter_cnt
        state['user_count'] = self._user_cnt
        state['board'] = self._grid
        self._board_state_pub.publish(str(state))

    def _on_roi(self, data):
        if data.polygon.points:
            for idx, point in enumerate(data.polygon.points):
                self._roi_points[3 - idx] = [int(point.x), int(point.y)]

    def _on_camera(self, data):
        try:
            self.cv_image = self._bridge.imgmsg_to_cv2(data,
                                                       desired_encoding="bgr8")
            local_image = np.asarray(self.cv_image)
        except Exception:
            print 'OH NO - IMAGE WENT WRONG!!'

        self.subLock.acquire(True)
        self._np_image = deepcopy(local_image)
        self.subLock.release()

    def _on_gain_slider(self, pos):
        self._gain_slider = pos
        self._gain_set = True
        self._slider_time = rospy.Time.now()

    def _on_red_slider(self, pos):
        self._red_thresh = pos

    def _on_yellow_slider(self, pos):
        self._yellow_thresh = pos

    def _on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            width = self.cv_image.shape[0]
            height = self.cv_image.shape[1]
            for idx, points in enumerate(self._roi_points):
                if (x <= points[0] + 5 and x >= points[0] - 5
                    and y <= points[1] + 5 and y >= points[1] - 5):
                    self._roi_move = True
                    self._point_selected = idx

        elif event == cv2.EVENT_MOUSEMOVE and self._roi_move:
            self._roi_points[self._point_selected] = [x, y]

        elif event == cv2.EVENT_LBUTTONUP and self._roi_move:
            self._roi_move = False
