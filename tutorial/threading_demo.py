# .. -*- coding: utf-8 -*-
#
# ****************************************************
# threading.py - Threads in Python.
# ****************************************************
# Threads are easy to get subtly wrong.

# Library imports
# ===============
from threading import Thread, Lock
from time import sleep

# A class to produce numbers.
class NumberFactory(object):
    def __init__(self):
        self._number = 0

    def get_number(self):
        self._number += 1
        return self._number

# Create a class that counts.
class CounterThread(Thread):
    def __init__(self,
      # A lock, needed before getting a number.
      lock,
      # Time to wait before getting a new number, in seconds.
      wait_time_sec,
      # Class to get numbers from.
      number_factory):

        Thread.__init__(self)
        self._lock = lock
        self._wait_time_sec = wait_time_sec
        self._number_factory = number_factory

    def run(self):
        for index in range(10):
            sleep(self._wait_time_sec)
            print('{}: {}'.format(self.name, self._number_factory.get_number()))

def main():
    nf = NumberFactory()
    lock = Lock()
    ct1 = CounterThread(lock, 1.5, nf, name='Slow')
    ct2 = CounterThread(lock, 1.0, nf, name='Fast')
    ct1.run()
    ct2.run()

    ct1.join()
    ct2.join()

if __name__=='__main__':
    main()
