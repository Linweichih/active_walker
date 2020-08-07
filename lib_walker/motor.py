import serial
import configparser
import os
config = configparser.ConfigParser()
parent_dir = os.path.dirname(os.path.abspath(__file__))
config.read(parent_dir + '/device.cfg')


class Motor:
    def __init__(self, motor_name):
        self.com_port = int(config.get(motor_name, 'PortName'))
        self.baud_rate = int(config.get(motor_name, 'BaudRate'))
        self.time_out = int(config.get(motor_name, 'Read_timeout'))
        self.serial = serial.Serial(self.com_port, self.baud_rate, timeout=self.time_out)
        # set acceleration maximum
        self.send_cmd("AC10")
        self.send_cmd("DEC100")
        # try to send the cmd to motor
        for i in range(5):
            self.send_cmd("V0")
        print(motor_name, "'s serial initialize")

    def send_cmd(self, cmd):
        motor_cmd = cmd + "\r\n"
        if self.serial.isOpen():
            try:
                self.serial.write(motor_cmd)
            except serial.SerialException:
                print("Can not send motor message")
            try:
                self.serial.readline()
            except serial.SerialException:
                print("Can not send motor message")

        else:
            print("The serial is not open")

    def get_motor_pos(self):
        motor_cmd = "POS\r\n"
        if self.serial.isOpen():
            try:
                self.serial.write(motor_cmd)
            except serial.SerialException:
                print("Can not send motor message")
            try:
                ret_str = self.serial.readline()
            except serial.SerialException:
                print("Can not send motor message")
            return int(ret_str)
        else:
            print("The serial is not open")
