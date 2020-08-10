from lib_walker.state import *
from lib_walker.sensor import *
from lib_walker.motor import *
import math



class Walker:
    def __init__(self):
        self.cam = UsbCam()
        self.human_state = State()
        self.walker_state = State()
        self.left_cmd = 0
        self.right_cmd = 0
        self.walker_x = 0
        self.walker_y = 0
        self.walker_theta = 0
        self.left_motor = Motor('left_motor')
        self.right_motor = Motor('right_motor')
        self.time_previous = -1

        self.pulse_l = 0
        self.pulse_r = 0
        self.gear_ratio = float(config.get('motor_config', 'gear_ratio'))
        self.wheel_radius = float(config.get('motor_config', 'wheel_radius'))
        self.wheel_dist = float(config.get('motor_config', 'wheel_distance'))

    def run(self):
        while True:
            t_start = time.time()
            ret_flag, image_frame = self.cam.read()
            # get the shoe detection result from image and the POS information from encoder

            self.get_walker_information()
            human_x, human_y, human_theta = self.get_human_information(image_frame)

            self.human_state.update(human_x, human_y, human_theta)
            self.controller()

            print("Process a image and send command spend ", time.time() - t_start, "secs")
            self.time_previous = time.time()

    def controller(self):
        pass

    def get_walker_information(self):
        if self.time_previous is not -1:
            # not first time record
            pulse_l = self.left_motor.get_motor_pos()
            pulse_r = self.right_motor.get_motor_pos()
            timer_interval = time.time() - self.time_previous
            vl = (pulse_l - self.pulse_l) / 3000 / self.gear_ratio * 2 * math.pi / timer_interval
            vr = -(pulse_r - self.pulse_r) / 3000 / self.gear_ratio * 2 * math.pi / timer_interval  # notice the "minus"
            walker_v = self.wheel_radius / 2 * (vr + vl)
            walker_omega = self.wheel_radius / self.wheel_dist * (vr - vl)

        else:
            # first time record
            self.walker_x

    def get_human_information(self, image):
        human_x_walker_based = 0
        human_y_walker_based = 0
        human_theta_walker_based = 0
        return human_x, human_y, human_theta
