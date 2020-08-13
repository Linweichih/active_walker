from threading import Timer
import time


class timer(Timer):
    """
    Call the sensor get the sensing message every 0.05 second
    """
    def run(self):
        while not self.finished.is_set():
            self.function(*self.args, **self.kwargs)
            self.finished.wait(self.interval)


if __name__ == '__main__':
    # cam = UsbCam()
    # print(cam, "initialize success!!")

    def cam_cb(timer_name):
        print("hello ", timer_name, "timer's start time: ", time.time())
        time.sleep(0.06)
        print("hello ", timer_name, "timer's end time: ", time.time())

    def con_cb(timer_name):
        print("hello ", timer_name, "timer's start time: ", time.time())
        time.sleep(0.01)
        print("control command!!!")
        print("hello ", timer_name, "timer's end time: ", time.time())

    t1 = timer(0.03, cam_cb, ('usb_cam',))
    t2 = timer(0.1, con_cb, ('controller',))
    t1.daemon = True
    t2.daemon = True
    t2.start()
    t1.start()
    time.sleep(10)
    t1.cancel()
    t2.cancel()


