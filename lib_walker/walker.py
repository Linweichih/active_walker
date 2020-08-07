from lib_walker.state import *
from lib_walker.sensor import *
from lib_walker.motor import *


class Walker:
    def __init__(self):
        self.cam = UsbCam()
        self.human_state = State()
        self.walker_state = State()
        self.left_cmd = 0
        self.right_cmd = 0
        self.left_motor = Motor('left_motor')
        self.right_motor = Motor('right_motor')

    def run(self):
        while True:
            time_previous = time.time()
            ret, frame = self.cam.read()
            # process the image and turn it into human pose
            human_x = 0
            human_y = 0
            human_theta = 0
            time_interval = time.time() - time_previous
            self.human_state.update(human_x, human_y, human_theta, time_interval)
            self.controller()

            print("Process a image and send command spend ", time.time() - time_previous, "secs")

    def controller(self):
        pass

