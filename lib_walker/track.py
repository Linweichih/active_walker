import cv2
import numpy as np
import math
import configparser
import os
config = configparser.ConfigParser()
parent_dir = os.path.dirname(os.path.abspath(__file__))
config.read(parent_dir + '/device.cfg')
INPUT_IMG_SIZE = (int(config.get('usb_cam', 'image_height')), int(config.get('usb_cam', 'image_width')))


def box_head(box):

    x = [box[0][0], box[1][0], box[2][0], box[3][0]]
    y = [box[0][1], box[1][1], box[2][1], box[3][1]]
    ind = np.lexsort((x, y))
    tmp = np.zeros((4, 2))
    # print(x)
    # print(y)
    # print(tmp)
    index = 0
    for i in ind:
        tmp[index] = [x[i], y[i]]
        index += 1
    return tmp[2] / 2 + tmp[3] / 2


class ObjectTrack:
    def __init__(self):
        self.position = [0, 0]
        self.angle = 0.0
        self.kalman = cv2.KalmanFilter(6, 3)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]],
                                                 np.float32)
        self.kalman.transitionMatrix = np.array(
            [[1, 0, 0, 1, 0, 0], [0, 1, 0, 0, 1, 0], [0, 0, 1, 0, 0, 1], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32) * 0.001
        self.kalman.measurementNoiseCov = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float32) * 0.001
        # filter the image of feet in the rec that the feet will appear
        self.filter_kernel = np.zeros(INPUT_IMG_SIZE, np.uint8)
        self.filter_kernel[0:384, 100:400] = 255
        # set the open/close operation kernel
        self.open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (30, 30))
        self.close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 20))

    def update(self, mask):

        # pre-process the mask
        # mask = cv2.bitwise_and(mask, self.filter_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.close_kernel)
        # get the obb of the mask
        self.position, self.angle, bbox = self.find_center_angle(mask)
        # implement kalman filter
        current_measurement = np.array(
            [[np.float32(self.position[0])], [np.float32(self.position[1])], [np.float32(self.angle)]])
        # print(current_measurement)
        if self.position[0] > 0:
            self.kalman.correct(current_measurement)
        pose = self.kalman.predict()
        self.position[0] = pose[0]
        self.position[1] = pose[1]
        self.angle = pose[2]
        return self.position, self.angle, bbox

    def find_center_angle(self, mask):
        center = [0, 200]
        box = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        center[0] = self.kalman.statePost[0]
        center[1] = self.kalman.statePost[1]
        angle = self.kalman.statePost[2]
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Ensure that the contours will only be one set
        if len(cnts) > 1:
            cnts = cnts[-1]
        # clone = image.copy()
        if len(cnts):
            box = cv2.minAreaRect(cnts[0])
            box = np.int0(cv2.boxPoints(box))
            # print("box's value  :", box)
            # print("box's center  :", (box[1]+box[3])/2)
            center = (box[1] + box[3]) / 2
            # print("clone's size:", clone.size)
            # find box's front point
            side = box_head(box)
            angle = math.atan2((side[0] - center[0]), (side[1] - center[1]))

        return center, angle, box


