# .. -*- coding: utf-8 -*-
#
# ****************************************************
# threading.py - Threads in Python.
# ****************************************************
# Threads are easy to get subtly wrong.
#
# Library imports
# ===============
from threading import Thread, Lock
from time import sleep
#
# NumberFactory - A class to produce numbers.
# ===========================================
class NumberFactory(object):
    def __init__(self,
          # A Lock instance to enforce exclusive access.
          lock):

        self._number = 0
        self._lock = lock

    # Get a unique number from the factory.
    def get_number(self):
        # A context manager makes using a lock `easy <https://docs.python.org/2/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement>`_.
        with self._lock:
            self._number += 1
            sleep(0.5)
            return self._number
#
# CounterThread - a class that counts in a separate thread.
# =================================================================
class CounterThread(Thread):
    def __init__(self,
      # Time to wait before getting a new number, in seconds.
      wait_time_sec,
      # Class to get numbers from.
      number_factory,
      # Any extra args.
      **kwargs):

        # Thread.__init__ `must <https://docs.python.org/2/library/threading.html#threading.Thread>`_ be called. To invoke a parent, common syntax is ``Thread.__init__(self, kwargs)``. This has several problems, however; use the (awkward) `super <https://docs.python.org/2/library/functions.html#super>`_ syntax instead. Note that this syntax is much improved in Python 3 -- simply use `super().__init__(kwargs) <https://docs.python.org/3/library/functions.html#super>`_.
        super(CounterThread, self).__init__(**kwargs)
        self._wait_time_sec = wait_time_sec
        self._number_factory = number_factory

    # `This method <https://docs.python.org/2/library/threading.html#threading.Thread.run>`_ runs in a separate thread.
    def run(self):
        for index in range(10):
            sleep(self._wait_time_sec)
            print('{}: {}'.format(self.name, self._number_factory.get_number()))
#
# main -- run the threads
# =======================
def main():
    # Create two threads.
    #
    # Use a `lock <https://docs.python.org/2/library/threading.html#lock-objects>`_ to enforce exclusive access in the NumberFactory.
    lock = Lock()
    nf = NumberFactory(lock)
    ct1 = CounterThread(1.0, nf, name='Slow')
    ct2 = CounterThread(0.75, nf, name='Fast')

    # Run them.
    ct1.start()
    ct2.start()

    # Wait for both of them to finish.
    ct1.join()
    ct2.join()

if __name__=='__main__':
    main()
