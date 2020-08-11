import time


class State:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.time_stamp = -1
    """
    useless
    def update(self, x, y, theta):
        time_interval = time.time() - self.time_stamp
        self.time_stamp = time.time()
        self.v_x = (x-self.x)/time_interval
        self.v_y = (y - self.y) / time_interval
        self.omega = (theta - self.theta) / time_interval
        self.x = x
        self.y = y
        self.theta = theta    
    """





