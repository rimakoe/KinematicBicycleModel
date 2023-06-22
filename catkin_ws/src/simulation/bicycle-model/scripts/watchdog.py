from .timing import time_ms
import rospy
import threading

class Watchdog:
	# Give a lambda as argument and execute it on fail to publish a message or trigger something outside the watchdog class
    def __init__(self, do_something : callable, max_elapsed_time=2000):
        self.__do_something = do_something
        self.__max_elapsed_time = max_elapsed_time # ms
        self.__stop = False
        self.__is_alive = True
        self.__elapsed_time = time_ms()
        self.__init_thread()

    def check(self, stop : callable) -> bool:
        while(True):
            if(stop()): break
            dt = time_ms() - self.__elapsed_time
            if(dt > self.__max_elapsed_time):
                self.__is_alive = False 
                self.__do_something()
                break
            else:
                rospy.sleep(0.1)
    
    def reset(self):
        self.__stop = False
        self.__is_alive = True
        self.__elapsed_time = time_ms()
        self.__init_thread()

    def trigger(self):
        self.__elapsed_time = time_ms()

    def start(self):
        self.__thread.start()

    def stop(self):
        self.__stop = True

    def is_alive(self) -> bool:
        return self.__is_alive

    def __init_thread(self):
        self.__thread = threading.Thread(target=self.check, args=(lambda: self.__stop, ))
