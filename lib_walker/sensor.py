import cv2
import sys
import os
import configparser
import serial
import time
import matplotlib.pyplot as plt
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
        return force_temp_list

    def close(self):
        self.serial.close()


if __name__ == '__main__':
    """
    force = ForceSensor()
    t = 0
    while True:
        force_list = force.read_force_data()
        print(force_list)
        time.sleep(0.05)
        t += 1
        if t == 10:
            break
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



