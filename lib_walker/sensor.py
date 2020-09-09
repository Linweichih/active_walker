import cv2
import sys
import os
import configparser
import serial
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
        print("initialize success ")

    def read(self):
        ret, frame = self.cap.read()
        return ret, frame

    def release(self):
        self.cap.release()


class ForceSensor:
    def __init__(self):
        self.com_port = int(config.get("force_sensor", 'PortName'))
        self.baud_rate = int(config.get("force_sensor", 'BaudRate'))
        self.time_out = int(config.get("force_sensor", 'Read_timeout'))
        self.force_sensor = serial.Serial(self.com_port, self.baud_rate, timeout=self.time_out)

        try:
            self.serial = serial.Serial(self.com_port, self.baud_rate, timeout=self.time_out)
        except serial.SerialException:
            print("ForceSensor Connect to {} serial error!!", self.com_port)
            sys.exit()
        # calibration part (send 'p' command)

    def read_force_data(self):
        # get force part (send 'R' command)
        pass


if __name__ == '__main__':
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

