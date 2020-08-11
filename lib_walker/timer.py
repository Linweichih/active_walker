from threading import Timer, Thread, Event
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
    def cb(time_interval):
        print("hello ", time_interval, "timer'time: ", time.time())

    t = timer(0.02, cb, (0.2,))
    t2 = timer(0.01, cb, (0.1,))
    t2.start()
    t.start()
    time.sleep(0.1)
    t.cancel()
    t2.cancel()


