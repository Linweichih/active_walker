from lib_walker.state import *
from lib_walker.sensor import *
from lib_walker.motor import *
from lib_walker.timer import *
# from lib_walker.shoe_detection import *
import math


class Walker:
    def __init__(self):
        self.MAX_W = float(config.get('motor_config', 'max_omega'))
        self.MAX_V = float(config.get('motor_config', 'max_velocity'))
        self.K_1 = float(config.get('controller_config', 'K_1'))
        self.K_2 = float(config.get('controller_config', 'K_2'))
        self.K_3 = float(config.get('controller_config', 'K_3'))
        self.K_4 = float(config.get('controller_config', 'K_4'))
        self.DIST_MAX = float(config.get('controller_config', 'DIST_MAX'))

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
        self.motor_semaphore = Semaphore(1)
        self.data_semaphore = Semaphore(1)
        self.time_previous = -1
        self.img_num = 0
        self.pulse_l = 0
        self.pulse_r = 0
        self.gear_ratio = float(config.get('motor_config', 'gear_ratio'))
        self.wheel_radius = float(config.get('motor_config', 'wheel_radius'))
        self.wheel_dist = float(config.get('motor_config', 'wheel_distance'))
        self.cam_timer = timer(0.01, self.image_process)
        self.cam_timer.daemon = True
        self.encoder_timer = timer(0.05, self.get_walker_information)
        self.cam_timer.daemon = True
        self.command_timer = timer(0.09, self.controller)
        self.command_timer.daemon = True
        # self.shoe_detection = ShoeDetection()
        print("walker init success!!")

    def run(self):
        self.cam_timer.start()
        self.encoder_timer.start()
        while True:
            t_start = time.time()
            # get the shoe detection result from image and the POS information from encoder

            # self.get_walker_information()
            # Controller Part Start here
            time.sleep(20)
            self.cam_timer.cancel()
            self.encoder_timer.cancel()
            print("Process a image and send command spend ", time.time() - t_start, "secs")
            self.time_previous = time.time()
            break

    def image_process(self):
        cv2.namedWindow('Read_image', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.namedWindow('Processed_image', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        ret_flag, image_frame = self.cam.read()
        if ret_flag:
            cv2.imshow('Read_image', image_frame)
            keyboard_ret = cv2.waitKey(1)
            if keyboard_ret == 27:
                self.cam_timer.cancel()
        # process the shoe detection
        """
        pro_image = self.shoe_detection.detect(image_frame)
        cv2.imshow('Processed_image', pro_image)
        """

    def controller(self):
        timer_interval = 0.09
        if self.data_semaphore.acquire():
            robot_vel = self.walker_state.v
            robot_angle_vel = self.walker_state.omega
            self.data_semaphore.release()

        accel = -self.K_1 * (robot_vel - desired_vel) - self.K_2 * (human_dist - desired_dist)
        angle_accel = -self.K_3 * (robot_angle_vel - desired_angle_vel) - self.K_4 * (human_angle - desired_angle)
        v = robot_vel + accel * timer_interval
        omega = robot_angle_vel + angle_accel * timer_interval
        if v > self.MAX_V:
            v = self.MAX_V
        if omega > self.MAX_W:
            omega = self.MAX_W
        desire_rpml = (2 * v - omega * self.wheel_dist) / (2 * self.wheel_dist) / math.pi / 2 * 60 * self.gear_ratio
        desire_rpmr = (2 * v + omega * self.wheel_dist) / (2 * self.wheel_dist) / math.pi / 2 * 60 * self.gear_ratio
        right_cmd = "V" + str(-1*int(desire_rpmr))
        left_cmd = "V" + str(int(desire_rpml))

        if self.motor_semaphore.acquire():
            self.left_motor.send_cmd(left_cmd)
            self.right_motor.send_cmd(right_cmd)
            self.motor_semaphore.release()

    def get_walker_information(self):
        if self.walker_state.time_stamp != -1:
            # not first time record
            if self.motor_semaphore.acquire():
                pulse_l = self.left_motor.get_motor_pos()
                pulse_r = self.right_motor.get_motor_pos()
                self.motor_semaphore.release()
            if self.data_semaphore.release():
                timer_interval = time.time() - self.walker_state.time_stamp
                self.data_semaphore.release()

            vl = (pulse_l - self.pulse_l) / 3000 / self.gear_ratio * 2 * math.pi / timer_interval
            vr = -(pulse_r - self.pulse_r) / 3000 / self.gear_ratio * 2 * math.pi / timer_interval  # notice the "minus"

            if self.data_semaphore.release():
                self.walker_state.v = self.wheel_radius / 2 * (vr + vl)
                self.walker_state.omega = self.wheel_radius / self.wheel_dist * (vr - vl)
                self.walker_state.x += self.walker_state.v * math.cos(self.walker_state.theta) * timer_interval
                self.walker_state.y += self.walker_state.v * math.sin(self.walker_state.theta) * timer_interval
                self.walker_state.theta += self.walker_state.omega * timer_interval
                self.walker_state.time_stamp = time.time()
                self.data_semaphore.release()

        else:
            # first time record the encoder information
            if self.data_semaphore.acquire():
                self.walker_state.time_stamp = time.time()
                self.data_semaphore.release()
"""
    def get_human_information(self, image):
            human_x_walker_based = 0
            human_y_walker_based = 0
            human_theta_walker_based = 0
            return human_x, human_y, human_theta
"""

