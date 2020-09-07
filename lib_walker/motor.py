import serial
import configparser
import os
import time
import sys
import math
import numpy as np
config = configparser.ConfigParser()
parent_dir = os.path.dirname(os.path.abspath(__file__))
config.read(parent_dir + '/device.cfg')


class MotorSerial:
    def __init__(self):
        self.com_port = config.get('motor_serial', 'PortName')
        self.baud_rate = int(config.get('motor_serial', 'BaudRate'))
        self.time_out = int(config.get('motor_serial', 'Read_timeout'))
        self.right_motor_node_name = config.get('motor_serial', 'right_motor_node_name')
        self.left_motor_node_name = config.get('motor_serial', 'left_motor_node_name')
        try:
            self.serial = serial.Serial(self.com_port, self.baud_rate, timeout=self.time_out)
        except serial.SerialException:
            print("Connect to {} serial error!!", self.com_port)
            sys.exit()
        # enable the driver of the motor
        self.send_cmd("left_motor", "EN")
        self.send_cmd("right_motor", "EN")
        # set acceleration and deceleration
        self.send_cmd("left_motor", "AC5")     # almost 0.3 m/s^2
        self.send_cmd("left_motor", "DEC10")    # almost -0.3 m/s^2
        self.send_cmd("right_motor", "AC5")
        self.send_cmd("right_motor", "DEC10")
        # try to send the cmd to motor
        for i in range(5):
            self.send_cmd("right_motor", "V0")
            self.send_cmd("left_motor", "V0")

        print('motor_serial', "'s serial initialize")

    def send_cmd(self, motor_name, cmd):
        if motor_name == 'right_motor':
            motor_cmd = str(self.right_motor_node_name) + cmd + "\n\r\0"
        else:
            motor_cmd = str(self.left_motor_node_name) + cmd + "\n\r\0"
        if self.serial.isOpen():
            try:
                self.serial.write(motor_cmd.encode())
            except serial.SerialException:
                print("Can not send motor message")
            try:
                ret_str = self.serial.readline()
                if 'OK'.encode() not in ret_str:
                    print('Command is not set to the driver and return OK ')
            except serial.SerialException:
                print("Can not send motor message")

        else:
            print("The serial is not open")

    def get_motor_pos(self, motor_name):
        if motor_name == 'right_motor':
            motor_cmd = str(self.right_motor_node_name) + "POS\n\r\0"
        else:
            motor_cmd = str(self.left_motor_node_name) + "POS\n\r\0"
        if self.serial.isOpen():
            try:
                self.serial.write(motor_cmd.encode())
            except serial.SerialException:
                print("Can not send motor message")
            try:
                ret_str = self.serial.readline()
                return int(ret_str)
            except serial.SerialException:
                print("Can not send motor message")

        else:
            print("The serial is not open")

    def close(self):
        self.send_cmd("right_motor", "V0")
        self.send_cmd("left_motor", "V0")
        self.send_cmd("right_motor", "DI")
        self.send_cmd("left_motor", "DI")
        self.serial.close()


if __name__ == '__main__':
    motor = MotorSerial()
    mo_v = 0.3
    mo_omega = 0
    desire_l = (2 * mo_v - mo_omega * 0.6) / (2 * 0.0625) / math.pi / 2 * 60 * 14
    desire_r = (2 * mo_v + mo_omega * 0.6) / (2 * 0.0625) / math.pi / 2 * 60 * 14
    test_right_cmd = "V" + str(-1 * int(desire_r))
    test_left_cmd = "V" + str(int(desire_l))
    pre_pulse_l = motor.get_motor_pos("left_motor")
    pre_pulse_r = motor.get_motor_pos("right_motor")
    motor.send_cmd("left_motor", test_left_cmd)
    motor.send_cmd("right_motor", test_right_cmd)
    time.sleep(3)
    #pulse_l = motor.get_motor_pos("left_motor")
    #pulse_r = motor.get_motor_pos("right_motor")
    #print(pulse_r-pre_pulse_r, pulse_l-pre_pulse_l)
    motor.close()
    sys.exit()
