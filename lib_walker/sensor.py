import cv2
import sys
import os
import configparser
import serial
import numpy as np
import time
import matplotlib.pyplot as plt
from threading import Thread
from matplotlib.animation import FuncAnimation
config = configparser.ConfigParser()
parent_dir = os.path.dirname(os.path.abspath(__file__))
config.read(parent_dir + '/device.cfg')


class UsbCam:
    def __init__(self):
        """
        initialize the video camera  and read the first frame
        """
        self.cap = cv2.VideoCapture(int(config.get('usb_cam', 'cam_src')))
        self.cap.set(4, int(config.get('usb_cam', 'image_width')))
        self.cap.set(3, int(config.get('usb_cam', 'image_height')))
        if self.cap.isOpened() is False:
            self.cap.open()
        print("initialize success ")

    def read(self):
        ret, frame = self.cap.read()
        return ret, frame

    def release(self):
        if self.cap.isOpened():
            self.cap.release()


class ForceSensor:
    def __init__(self):
        self.com_port = config.get("force_sensor", 'PortName')
        self.baud_rate = int(config.get("force_sensor", 'BaudRate'))
        self.time_out = int(config.get("force_sensor", 'Read_timeout'))
        self.cal_list = []
        self.offset_list = []
        self.force_filter = cv2.KalmanFilter(6, 6)
        self.force_filter.measurementMatrix = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32)
        self.force_filter.transitionMatrix = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32)
        self.force_filter.measurementNoiseCov = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32) * 0.01
        self.force_filter.processNoiseCov = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32) * 0.0001
        try:
            self.serial = serial.Serial(self.com_port, self.baud_rate, timeout=self.time_out)
        except serial.SerialException:
            print("ForceSensor Connect to {} serial error!!".format(self.com_port))
            sys.exit()
        # calibration part (send 'p' command)
        if self.serial.isOpen():
            try:
                self.serial.write('p'.encode())
            except serial.SerialException:
                print("Can not send motor message")
            try:
                cal_list = self.serial.readline()
            except serial.SerialException:
                print("Can not send motor message")
        else:
            print("The serial is not open")
        cal_list = cal_list.decode()
        cal_str_list = cal_list.split(',')

        for c_str in cal_str_list:
            self.cal_list.append(float(c_str))
        print("cal_list", self.cal_list)
        # get the offset force list
        if self.serial.isOpen():
            try:
                self.serial.write('R'.encode())
            except serial.SerialException:
                print("Can not send motor message")
            try:
                ret_str = self.serial.readline()
            except serial.SerialException:
                print("Can not send motor message")
        else:
            print("The serial is not open")
        ret_str = ret_str.decode()
        # print(ret_str)
        for i in range(6):
            self.offset_list.append(int(ret_str[4 * i + 1:4 * i + 5], 16))
        print("offset_list:", self.offset_list)

    def read_force_data(self):
        force_temp_list = []
        # get force part (send 'R' command)
        if self.serial.isOpen():
            try:
                self.serial.write('R'.encode())
            except serial.SerialException:
                print("Can not send motor message")
            try:
                ret_str = self.serial.readline()
            except serial.SerialException:
                print("Can not send motor message")
        else:
            print("The serial is not open")
        ret_str = ret_str.decode()
        # print(ret_str)
        for i in range(6):
            force_temp_list.append((int(ret_str[4*i+1:4*i+5], 16)-self.offset_list[i]) / self.cal_list[i])
        current_measurement = np.array(
            [[np.float32(force_temp_list[0])], [np.float32(force_temp_list[1])], [np.float32(force_temp_list[2])],
             [np.float32(force_temp_list[3])], [np.float32(force_temp_list[4])], [np.float32(force_temp_list[5])]])
        self.force_filter.correct(current_measurement)
        filter_force = self.force_filter.predict()
        return filter_force

    def close(self):
        self.serial.close()


class LiveGraph:
    def __init__(self):
        self.x_data, self.y_data = [], []
        self.figure = plt.figure()
        self.force = ForceSensor()
        self.line, = plt.plot(self.x_data, self.y_data)
        self.animation = FuncAnimation(self.figure, self.update, interval=1000)
        self.th = Thread(target=self.thread_f, daemon=True)
        self.th.start()

    def update(self, frame):
        self.line.set_data(self.x_data, self.y_data)
        self.figure.gca().relim()
        self.figure.gca().autoscale_view()
        return self.line,

    def show(self):
        plt.show()

    def thread_f(self):
        t = 0
        while True:
            self.x_data.append(t)
            force_list = self.force.read_force_data()
            t += 0.1
            self.y_data.append(force_list[2])
            time.sleep(0.1)


if __name__ == '__main__':
    g = LiveGraph()
    g.show()
    """
    cv2.namedWindow('image_win', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
    cam = UsbCam()
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
        # cv2.destroyAllWindows()
        sys.exit()
        """



