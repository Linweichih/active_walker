import serial
import configparser
import os
config = configparser.ConfigParser()
parent_dir = os.path.dirname(os.path.abspath(__file__))
config.read(parent_dir + '/device.cfg')

class motor:
    def __init__(self):
        pass


