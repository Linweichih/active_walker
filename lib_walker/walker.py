from lib_walker.state import *
from lib_walker.sensor import *
from lib_walker.motor import *
from lib_walker.timer import *
from lib_walker.shoe_detection import *
from threading import Semaphore
import math


def img2real_transform(human_pos_walker_frame, human_ang_walker_frame):
    """
        real transform will depend on the camera placed
    """
    return human_pos_walker_frame, human_ang_walker_frame


DIST_MAX = float(config.get('human_safety_parameter', 'DIST_MAX'))
DIST_MIN = float(config.get('human_safety_parameter', 'DIST_MIN'))
ANGLE_MAX = float(config.get('human_safety_parameter', 'ANGLE_MAX'))
ANGLE_MIN = float(config.get('human_safety_parameter', 'ANGLE_MIN'))


def get_desired_pose(human_dist, human_angle):
    if human_dist > DIST_MAX:
        desired_dist = DIST_MAX
    elif human_dist < DIST_MIN:
        desired_dist = DIST_MIN
    else:
        desired_dist = human_dist
    if human_angle > ANGLE_MAX:
        desired_angle = ANGLE_MAX
    elif human_angle < ANGLE_MIN:
        desired_angle = ANGLE_MIN
    else:
        desired_angle = human_dist
    return desired_dist, desired_angle


class Walker:
    def __init__(self):
        self.MAX_W = float(config.get('motor_config', 'max_omega'))
        self.MAX_V = float(config.get('motor_config', 'max_velocity'))
        self.K_1 = float(config.get('controller_config', 'K_1'))
        self.K_2 = float(config.get('controller_config', 'K_2'))
        self.K_3 = float(config.get('controller_config', 'K_3'))
        self.K_4 = float(config.get('controller_config', 'K_4'))

        self.cam = UsbCam()
        self.human_state = State()
        self.walker_state = State()
        self.left_cmd = 0
        self.right_cmd = 0
        self.walker_x = 0
        self.walker_y = 0
        self.walker_theta = 0
        self.motor_serial = MotorSerial()
        self.motor_semaphore = Semaphore(1)
        self.walker_data_semaphore = Semaphore(1)
        self.human_data_semaphore = Semaphore(1)
        self.time_previous = -1
        self.img_num = 0
        self.pulse_l = 0
        self.pulse_r = 0
        self.gear_ratio = float(config.get('motor_config', 'gear_ratio'))
        self.wheel_radius = float(config.get('motor_config', 'wheel_radius'))
        self.wheel_dist = float(config.get('motor_config', 'wheel_distance'))
        self.cam_timer = timer(0.03, self.image_process)
        self.cam_timer.daemon = True
        self.encoder_timer = timer(0.05, self.get_walker_information)
        self.cam_timer.daemon = True
        self.command_timer = timer(0.09, self.controller)
        self.command_timer.daemon = True
        self.shoe_detection = ShoeDetection()
        print("walker init success!!")

    def run(self):
        self.cam_timer.start()
        time.sleep(10)
        # wait 5 secs to let the camera track the feet
        self.encoder_timer.start()
        self.command_timer.start()
        while True:
            time.sleep(1)
            # get the shoe detection result from image and the POS information from encoder
            # Controller Part Start here

    def image_process(self):
        cv2.namedWindow('Read_image', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.namedWindow('Processed_image', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.namedWindow('detection_mask', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        ret_flag, image_frame = self.cam.read()
        if ret_flag:
            cv2.imshow('Read_image', image_frame)
            keyboard_ret = cv2.waitKey(1)
            if keyboard_ret == 27:
                if self.command_timer.is_alive():
                    self.command_timer.cancel()
                if self.encoder_timer.is_alive():
                    self.encoder_timer.cancel()
                self.cam_timer.cancel()
        # process the shoe detection
        pro_image, mask, human_pos_walker_frame, human_ang_walker_frame = self.shoe_detection.detect(image_frame)
        cv2.imshow('Processed_image', pro_image)
        cv2.imshow('detection_mask', mask)
        human_pos, human_ang = img2real_transform(human_pos_walker_frame, human_ang_walker_frame)
        time_stamp = -2
        if self.human_data_semaphore.acquire():
            time_stamp = self.human_state.time_stamp
            self.human_data_semaphore.release()
        if time_stamp == -1:
            # first time record the human pose information
            if self.human_data_semaphore.acquire():
                self.human_state.time_stamp = time.time()
                self.human_data_semaphore.release()
        elif time_stamp == -2:
            print("time_stamp semaphore is error")
            sys.exit()
        else:
            if self.human_data_semaphore.acquire():
                self.human_state.v = human_pos[1] - self.human_state.y
                self.human_state.omega = human_ang - self.human_state.theta
                self.human_state.y = human_pos[1]
                self.human_state.x = human_pos[0]
                self.human_state.theta = human_ang
                self.human_state.time_stamp = time.time()
                self.human_data_semaphore.release()

    def controller(self):
        timer_interval = 0.09
        if self.walker_data_semaphore.acquire():
            robot_vel = self.walker_state.v
            robot_angle_vel = self.walker_state.omega
            self.walker_data_semaphore.release()
        # Position control
        desired_vel = robot_vel
        desired_angle_vel = robot_angle_vel

        if self.human_data_semaphore.acquire():
            human_dist = self.human_state.y
            human_angle = self.human_state.theta
            self.human_data_semaphore.release()
        desired_dist, desired_angle = get_desired_pose(human_dist, human_angle)
        accel = -self.K_1 * (robot_vel - desired_vel) - self.K_2 * (human_dist - desired_dist)
        angle_accel = -self.K_3 * (robot_angle_vel - desired_angle_vel) - self.K_4 * (human_angle - desired_angle)
        print("accel:", accel, "angle_accel", angle_accel)
        v = robot_vel + accel * timer_interval
        omega = robot_angle_vel + angle_accel * timer_interval
        if v > self.MAX_V:
            v = self.MAX_V
        if omega > self.MAX_W:
            omega = self.MAX_W
        desire_rpm_l = (2 * v - omega * self.wheel_dist) / (2 * self.wheel_dist) / math.pi / 2 * 60 * self.gear_ratio
        desire_rpm_r = (2 * v + omega * self.wheel_dist) / (2 * self.wheel_dist) / math.pi / 2 * 60 * self.gear_ratio
        right_cmd = "V" + str(-1*int(desire_rpm_r))
        left_cmd = "V" + str(int(desire_rpm_l))

        if self.motor_semaphore.acquire():
            print("l_cmd:", left_cmd, "r_cmd:", right_cmd)
            self.motor_serial.send_cmd("left_motor", left_cmd)
            self.motor_serial.send_cmd("right_motor", right_cmd)
            self.motor_semaphore.release()

    def get_walker_information(self):
        time_stamp = -2
        if self.walker_data_semaphore.acquire():
            time_stamp = self.walker_state.time_stamp
            self.walker_data_semaphore.release()
        if time_stamp != -1:
            # not first time record
            if self.motor_semaphore.acquire():
                print("get encoder information!!")
                pulse_l = self.motor_serial.get_motor_pos("left_motor")
                pulse_r = self.motor_serial.get_motor_pos("right_motor")
                self.motor_semaphore.release()

            timer_interval = time.time() - time_stamp

            vl = (pulse_l - self.pulse_l) / 3000 / self.gear_ratio * 2 * math.pi / timer_interval
            vr = -(pulse_r - self.pulse_r) / 3000 / self.gear_ratio * 2 * math.pi / timer_interval  # notice the "minus"

            if self.walker_data_semaphore.release():
                self.walker_state.v = self.wheel_radius / 2 * (vr + vl)
                self.walker_state.omega = self.wheel_radius / self.wheel_dist * (vr - vl)
                self.walker_state.x += self.walker_state.v * math.cos(self.walker_state.theta) * timer_interval
                self.walker_state.y += self.walker_state.v * math.sin(self.walker_state.theta) * timer_interval
                self.walker_state.theta += self.walker_state.omega * timer_interval
                self.walker_state.time_stamp = time.time()
                self.walker_data_semaphore.release()
        elif time_stamp == -2:
            print("time_stamp semaphore is error")
            sys.exit()
        else:
            # first time record the encoder information
            if self.walker_data_semaphore.acquire():
                self.walker_state.time_stamp = time.time()
                self.walker_data_semaphore.release()

