from lib_walker.state import *
from lib_walker.sensor import *
from lib_walker.motor import *
from lib_walker.timer import *
from lib_walker.shoe_detection import *
from lib_walker.plot import *
from threading import Semaphore
import math
import pandas as pd


DIST_MAX = float(config.get('human_safety_parameter', 'DIST_MAX'))
DIST_MIN = float(config.get('human_safety_parameter', 'DIST_MIN'))
ANGLE_MAX = float(config.get('human_safety_parameter', 'ANGLE_MAX'))
ANGLE_MIN = float(config.get('human_safety_parameter', 'ANGLE_MIN'))


def img2real_transform(human_pos_walker_frame, human_ang_walker_frame):
    """
        real transform will depend on the camera placed (calibration)
        /2500*6 is calibration by rule of thumb
    """
    human_pos = np.zeros(2)
    human_pos_walker_frame[1] = 384 - human_pos_walker_frame[1]
    human_pos[0] = 0
    human_pos[1] = human_pos_walker_frame[1] / 750*2*6/7
    return human_pos, float(human_ang_walker_frame)


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
        desired_angle = human_angle
    return desired_dist, desired_angle


class Walker:
    def __init__(self):
        self.start_reg = 0
        self.start_record = 0
        self.time_start = 0
        self.stop_sys_flag = 0
        self.no_foot_flag = True
        self.base_y = 383
        self.pre_rel_v = 0
        self.pre_dist_error = 0
        self.pre_angle_error = 0
        self.pre_rel_omega = 0
        # init csv file to write
        self.walker_data = {'time': [],
                            'x': [], 'y': [], 'path': [], 'theta': [],
                            'v': [], 'omega': []}
        self.human_data = {'time': [],
                           'x': [], 'y': [], 'theta': [],
                           'v': [], 'omega': [],
                           'x_force': [], 'y_force': [], 'z_force': [],
                           'x_torque': [], 'y_torque': [], 'z_torque': []}

        self.MAX_W = float(config.get('motor_config', 'max_omega'))
        self.MAX_V = float(config.get('motor_config', 'max_velocity'))
        self.MAX_AC = self.MAX_V * 4
        self.MAX_A_AC = self.MAX_W * 4
        self.K_1 = float(config.get('controller_config', 'K_1'))
        self.K_2 = float(config.get('controller_config', 'K_2'))
        self.K_3 = float(config.get('controller_config', 'K_3'))
        self.K_4 = float(config.get('controller_config', 'K_4'))
        self.gear_ratio = float(config.get('motor_config', 'gear_ratio'))
        self.wheel_radius = float(config.get('motor_config', 'wheel_radius'))
        self.wheel_dist = float(config.get('motor_config', 'wheel_distance'))

        # human pos track filter parameter set
        self.human_pos_filter = cv2.KalmanFilter(6, 2)
        self.human_pos_filter.measurementMatrix = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0]], np.float32)
        self.human_pos_filter.transitionMatrix = np.array(
            [[1, 0, 1, 0, 0, 0],
             [0, 1, 0, 1, 0, 0],
             [0, 0, 1, 0, 1, 0],
             [0, 0, 0, 1, 0, 1],
             [0, 0, 0, 0, 0, 1],
             [0, 0, 0, 0, 0, 1]], np.float32)
        self.human_pos_filter.measurementNoiseCov = np.array(
            [[1, 0],
             [0, 1]], np.float32) * 0.01
        self.human_pos_filter.processNoiseCov = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32) * 0.00001

        self.shoe_detection = ShoeDetection()
        self.force_sensor = ForceSensor()
        self.motor_serial = MotorSerial()
        self.cam = UsbCam()
        self.human_state = State()
        self.walker_state = State()
        self.left_cmd = 0
        self.right_cmd = 0
        self.walker_x = 0
        self.walker_y = 0
        self.walker_theta = 0
        # semaphore set
        self.motor_semaphore = Semaphore(1)
        self.walker_data_semaphore = Semaphore(1)
        self.human_data_semaphore = Semaphore(1)

        self.time_previous = -1
        self.img_num = 0
        self.pulse_l = 0
        self.pulse_r = 0

        # timer set
        self.cam_timer = timer(0.03, self.get_human_information)
        self.cam_timer.daemon = True
        self.encoder_timer = timer(0.06, self.get_walker_information)
        self.cam_timer.daemon = True
        self.command_timer = timer(0.09, self.controller)
        self.command_timer.daemon = True

        print("walker init success!!")

    def run(self):
        self.cam_timer.start()
        self.time_start = time.time()
        time.sleep(2)

        # make the motor be push with human hand
        # self.motor_serial.send_cmd("right_motor", "DI")
        # self.motor_serial.send_cmd("left_motor", "DI")
        self.encoder_timer.start()
        # wait for the camera track the feet
        while self.start_reg == 0:
            time.sleep(0.2)
        time.sleep(6)
        print("Controller start !!")
        self.command_timer.start()

        while True:
            time.sleep(1)
            if self.stop_sys_flag == 1:
                if self.command_timer.is_alive():
                    print("shut down the command timer")
                    self.command_timer.cancel()
                if self.motor_semaphore.acquire():
                    self.motor_serial.close()
                    self.motor_semaphore.release()
                print("Close motor serial!!")
                if self.encoder_timer.is_alive():
                    print("shut down the encoder_timer")
                    self.encoder_timer.cancel()
                time.sleep(1)
                break
        walker_df = pd.DataFrame(self.walker_data)
        human_df = pd.DataFrame(self.human_data)
        os.mkdir("./Data_Result/test/" + time.strftime("%b_%d_%H_%M_%S", time.localtime()))
        walker_df.to_csv("./Data_Result/test/" + time.strftime("%b_%d_%H_%M_%S", time.localtime()) + "/walker_data.csv",
                         sep=',')
        human_df.to_csv("./Data_Result/test/" + time.strftime("%b_%d_%H_%M_%S", time.localtime()) + "/human_data.csv",
                        sep=',')
        print("\nRecord the data's csv file!!")

    def get_human_information(self):
        # cv2.namedWindow('Read_image', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.namedWindow('Processed_image', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.namedWindow('detection_mask', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        ret_flag, image_frame = self.cam.read()
        if ret_flag:
            # cv2.imshow('Read_image', image_frame)
            keyboard_ret = cv2.waitKey(1)
            if keyboard_ret == 27:
                time.sleep(1)
                self.stop_sys_flag = 1
                self.cam.release()
                print("Close the camera !!")
                print("shut down the cam_timer")
                self.cam_timer.cancel()

        # process the shoe detection
        pro_image, mask, human_pos_walker_frame, human_ang_walker_frame = self.shoe_detection.detect(image_frame)
        if human_pos_walker_frame[0] == -1:
            self.no_foot_flag = True
        else:
            self.no_foot_flag = False
        # image_frame = cv2.resize(image_frame, (512, 384))
        mask = cv2.resize(mask, (512, 384))
        # mask = np.concatenate((image_frame, mask), axis=1)

        human_pos, human_ang = img2real_transform(human_pos_walker_frame, human_ang_walker_frame)
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (0, 50)
        fontScale = 1
        fontColor = (255, 255, 0)
        lineType = 2

        # human_pos[1] = 384 - human_pos[1]
        # human_ang = -1 * human_ang
        force_data_list = self.force_sensor.read_force_data()
        if self.walker_data_semaphore.acquire():
            robot_vel = self.walker_state.v
            robot_path = self.walker_state.path
            robot_angle_vel = self.walker_state.omega
            self.walker_data_semaphore.release()
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
            print("time_stamp semaphore has error")
            sys.exit()
        else:
            if self.human_data_semaphore.acquire():
                human_pos.astype(float)
                time_interval = time.time() - self.human_state.time_stamp

                y = human_pos[1]
                x = human_pos[0]
                theta = human_ang
                y = robot_path - y
                current_measurement = np.array(
                    [[np.float32(y)], [np.float32(theta)]])
                self.human_pos_filter.correct(current_measurement)
                pose = self.human_pos_filter.predict()
                y = pose[0]
                theta = pose[1]
                v = pose[2]
                omega = pose[3]
                v = v / time_interval
                omega = omega / time_interval
                accel = pose[4] / time_interval / time_interval
                angle_accel = pose[5] / time_interval / time_interval
                # v = robot_vel - v
                self.human_state.v = v
                self.human_state.omega = omega
                self.human_state.accel = accel
                self.human_state.angle_accel = angle_accel
                self.human_state.y = y
                self.human_state.x = x
                self.human_state.theta = theta

                self.human_state.x_force = force_data_list[0]
                self.human_state.y_force = force_data_list[1]
                self.human_state.z_force = force_data_list[2]
                self.human_state.x_torque = force_data_list[3]
                self.human_state.y_torque = force_data_list[4]
                self.human_state.z_torque = force_data_list[5]

                self.human_state.time_stamp = time.time()

                if abs(v) < 0.01 and self.start_record == 0 and time.time() - self.time_start > 4:
                    # self.base_y = self.human_state.y
                    self.start_record = 1

                if self.start_record == 1:
                    """
                    print("robot_vel", robot_vel, "robot_angle_vel:", robot_angle_vel,
                          "\nhuman_v:", v, "human_omega:", robot_angle_vel - omega,
                          "\nhuman_y:", y, "\nrobot_y:", robot_path)
                    """
                    self.human_data['x'].append(0)
                    try:
                        self.human_data['y'].append(y.__float__().__format__(".3f"))
                    except TypeError:
                        print("detect TypeError")
                        self.human_data['y'].append(y)
                    try:
                        self.human_data['theta'].append(theta.__float__().__format__(".4f"))
                    except TypeError:
                        print("detect theta TypeError", theta)
                        self.human_data['theta'].append(theta)
                    try:
                        self.human_data['v'].append(v.__float__().__format__(".2f"))
                    except TypeError:
                        print(v)
                        self.human_data['v'].append(v)
                    try:
                        self.human_data['omega'].append((robot_angle_vel - omega).__float__().__format__(".3f"))
                    except TypeError:
                        print("detect omega TypeError", omega)
                        self.human_data['omega'].append(omega)
                    self.human_data['x_force'].append(self.human_state.x_force.__float__().__format__(".4f"))
                    self.human_data['y_force'].append(self.human_state.y_force.__float__().__format__(".4f"))
                    self.human_data['z_force'].append(self.human_state.z_force.__float__().__format__(".4f"))
                    self.human_data['x_torque'].append(self.human_state.x_torque.__float__().__format__(".4f"))
                    self.human_data['y_torque'].append(self.human_state.y_torque.__float__().__format__(".4f"))
                    self.human_data['z_torque'].append(self.human_state.z_torque.__float__().__format__(".4f"))
                    self.human_data['time'].append(format(self.human_state.time_stamp - self.time_start, ".2f"))
                    # print("human_v:", (robot_vel - v).__format__(".2f"),
                    #      "human_dist:", (y-self.base_y).__format__(".3f"))
                    try:
                        pro_image = cv2.putText(pro_image, str(y) + str(v), bottomLeftCornerOfText, font, fontScale,
                                                fontColor, lineType)
                    except:
                        print("ERROR")
                self.human_data_semaphore.release()
            self.start_reg = 1
            # write the pos number

        cv2.imshow('Processed_image', pro_image)
        cv2.imshow('detection_mask', mask)

    def controller(self):
        timer_interval = 0.09

        if self.walker_data_semaphore.acquire():
            robot_vel = self.walker_state.v
            robot_angle_vel = self.walker_state.omega
            robot_path = self.walker_state.path
            self.walker_data_semaphore.release()

        if self.human_data_semaphore.acquire():
            human_dist = self.human_state.y
            human_angle = self.human_state.theta
            human_v = self.human_state.v
            relative_omega = self.human_state.omega
            self.human_data_semaphore.release()

        human_relative_dist = robot_path - human_dist
        desired_dist, desired_angle = get_desired_pose(human_dist, human_angle)
        """
        human_dist = desired_dist
        human_angle = desired_angle
        """
        dist_error = human_relative_dist - desired_dist
        # de_dist_error = (dist_error - self.pre_dist_error) / timer_interval
        self.pre_dist_error = dist_error
        angle_error = human_angle - desired_angle
        # de_angle_error = (angle_error - self.pre_angle_error) / timer_interval
        self.pre_angle_error = angle_error
        relative_v = robot_vel - human_v
        # rel_a = (relative_v - self.pre_rel_v) / timer_interval
        self.pre_rel_v = relative_v
        # rel_angle_a = (relative_omega - self.pre_rel_omega) / timer_interval
        self.pre_rel_omega = relative_omega

        # original control law
        accel = -self.K_1 * relative_v - self.K_2 * dist_error
        angle_accel = -self.K_3 * relative_omega - self.K_4 * angle_error

        # constrain of acceleration
        if abs(accel) > self.MAX_AC:
            accel = self.MAX_AC * accel / abs(accel)
        if abs(angle_accel) > self.MAX_A_AC:
            angle_accel = self.MAX_A_AC * angle_accel / abs(angle_accel)
        v = robot_vel + accel * timer_interval
        omega = robot_angle_vel + angle_accel * timer_interval
        if abs(v) > self.MAX_V:
            v = self.MAX_V * v / abs(v)
        if abs(omega) > self.MAX_W:
            omega = self.MAX_W * omega / abs(omega)
        # make the walker stop when foot ars not detected
        if self.no_foot_flag:
            v = 0
            omega = 0

        print("distance error :", dist_error, "angle error :", human_angle - desired_angle,
              "vel_error:", relative_v, "omega_error:", relative_omega,
              "\naccel:", accel, "angle_accel", angle_accel,
              "walker_v:", robot_vel, "walker_omega:", robot_angle_vel,
              "\n", "Time:", time.time() - self.time_start)

        desire_rpm_l = (2 * v - omega * self.wheel_dist) / (2 * self.wheel_radius) / math.pi / 2 * 60 * self.gear_ratio
        desire_rpm_r = (2 * v + omega * self.wheel_dist) / (2 * self.wheel_radius) / math.pi / 2 * 60 * self.gear_ratio
        right_cmd = "V" + str(-1 * int(desire_rpm_r))
        left_cmd = "V" + str(int(desire_rpm_l))
        """
        print("desired_v:", v, "desired_w", omega, "Time:", time.time() - self.time_start,
              "desire_rpm_r:", desire_rpm_r, "desire_rpm_l:", desire_rpm_l)
        """

        if self.motor_semaphore.acquire():
            self.motor_serial.send_cmd("left_motor", left_cmd)
            self.motor_semaphore.release()
        if self.motor_semaphore.acquire():
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
                # print("get left_motor encoder information!!")
                pulse_l = self.motor_serial.get_motor_pos("left_motor")
                self.motor_semaphore.release()
            if self.motor_semaphore.acquire():
                # print("get right_motor encoder information!!")
                pulse_r = self.motor_serial.get_motor_pos("right_motor")
                self.motor_semaphore.release()
            timer_interval = time.time() - time_stamp
            if pulse_l == 'NAN':
                pulse_l = self.pulse_l
                self.stop_sys_flag = 1
            if pulse_r == 'NAN':
                pulse_r = self.pulse_r
            vl = (pulse_l - self.pulse_l) / 3000 / self.gear_ratio * 2 * math.pi / timer_interval
            vr = -(pulse_r - self.pulse_r) / 3000 / self.gear_ratio * 2 * math.pi / timer_interval  # notice the "minus"
            self.pulse_l = pulse_l
            self.pulse_r = pulse_r
            if self.walker_data_semaphore.acquire():
                # print("walker information:", self.walker_state.x, self.walker_state.y,
                # self.walker_state.theta*180/math.pi, self.walker_state.v, self.walker_state.omega,
                # self.walker_state.time_stamp - self.time_start) because the encoder will return the error message
                # make the velocity to more than 100
                if self.wheel_radius / 2 * (vr + vl) < 2:
                    v = self.wheel_radius / 2 * (vr + vl)
                    omega = self.wheel_radius / self.wheel_dist * (vr - vl)
                    x = self.walker_state.x + self.walker_state.v * math.cos(self.walker_state.theta) * timer_interval
                    y = self.walker_state.y + self.walker_state.v * math.sin(self.walker_state.theta) * timer_interval
                    path = self.walker_state.path + self.walker_state.v * timer_interval
                    theta = self.walker_state.theta + self.walker_state.omega * timer_interval
                    self.walker_state.v = v
                    self.walker_state.omega = omega
                    self.walker_state.path = path
                    self.walker_state.x = x
                    self.walker_state.y = y
                    self.walker_state.theta = theta
                    self.walker_state.time_stamp = time.time()

                    self.walker_data['x'].append(format(x, ".3f"))
                    self.walker_data['y'].append(format(y, ".3f"))
                    self.walker_data['path'].append(format(path, ".3f"))
                    self.walker_data['theta'].append(format(theta, ".4f"))
                    self.walker_data['v'].append(format(v, ".3f"))
                    self.walker_data['omega'].append(format(omega, ".3f"))
                    self.walker_data['time'].append(format(self.walker_state.time_stamp - self.time_start, ".2f"))
                    # print('walker_x:', format(x, ".3f"), 'walker_y:', format(y, ".3f"))
                self.walker_data_semaphore.release()

        elif time_stamp == -2:
            print("time_stamp semaphore is error")
            sys.exit()
        else:
            # first time record the encoder information
            if self.walker_data_semaphore.acquire():
                self.walker_state.time_stamp = time.time()
                self.walker_state.v = 0
                self.walker_state.omega = 0
                self.walker_state.x = 0
                self.walker_state.y = 0
                self.walker_state.path = 0
                self.walker_state.theta = math.pi / 2
                # self.time_start = self.walker_state.time_stamp
                self.walker_data_semaphore.release()


"""
    def motor_test(self):
        mo_v = 0.5
        mo_omega = 0
        desire_l = (2 * mo_v - mo_omega * 0.6) / (2 * 0.0625) / math.pi / 2 * 60 * 14
        desire_r = (2 * mo_v + mo_omega * 0.6) / (2 * 0.0625) / math.pi / 2 * 60 * 14
        test_right_cmd = "V" + str(-1 * int(desire_r))
        test_left_cmd = "V" + str(int(desire_l))
        if self.motor_semaphore.acquire():
            self.motor_serial.send_cmd("left_motor", test_left_cmd)
            self.motor_serial.send_cmd("right_motor", test_right_cmd)
            print("set command :", test_left_cmd, test_right_cmd)
            self.motor_semaphore.release()
        time.sleep(1)
        mo_v = 0.1
        mo_omega = 0
        desire_l = (2 * mo_v - mo_omega * 0.6) / (2 * 0.0625) / math.pi / 2 * 60 * 14
        desire_r = (2 * mo_v + mo_omega * 0.6) / (2 * 0.0625) / math.pi / 2 * 60 * 14
        test_right_cmd = "V" + str(-1 * int(desire_r))
        test_left_cmd = "V" + str(int(desire_l))
        if self.motor_semaphore.acquire():
            self.motor_serial.send_cmd("left_motor", test_left_cmd)
            self.motor_serial.send_cmd("right_motor", test_right_cmd)
            print("set command :", test_left_cmd, test_right_cmd)
            self.motor_semaphore.release()
        time.sleep(3)
"""
