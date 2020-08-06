from lib_walker import *
from lib_walker.sensor import *


class walker:
    def __init__(self):
        self.cam = usb_cam()

    def run(self):

        while True:
            time_previous = time.time()
            ret, frame = self.cam.read()
            # process the image and turn it into human pose


if __name__ == "__main__":
    wr = walker()
    wr.run()



