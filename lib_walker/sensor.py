import numpy as np
import cv2
import logging
from threading import Thread
import time
import sys
import os
import configparser
import serial
config = configparser.ConfigParser()
parent_dir = os.path.dirname(os.path.abspath(__file__))
config.read(parent_dir + '/device.cfg')


class usb_cam:
    def __init__(self):
        """
        initialize the video camera  and read the first frame
        """
        self.cap = cv2.VideoCapture(int(config.get('usb_cam', 'cam_src')))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(config.get('usb_cam', 'picture_width')))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config.get('usb_cam', 'picture_height')))
        print("initialize success ")

    def read(self):
        ret, frame = self.cap.read()
        return ret, frame


if __name__ == '__main__':
    cv2.namedWindow('image_win', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
    cam = usb_cam()
    try:
        while True:
            ret, frame = cam.read()
            if ret:
                cv2.imshow('image_win', frame)
                key = cv2.waitKey(1)

            else:
                print("fail to get USB CAM")
                break
    except KeyboardInterrupt:
        cam.cap.release()
        print("")
        print("User Pressed Keyboard ctrl-c")
        cv2.destroyAllWindows()
        sys.exit()
