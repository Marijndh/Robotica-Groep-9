import time

def waiting(duration):
    startDelay = time.clock_gettime_ns(0)
    while time.clock_gettime_ns(0) - startDelay < (duration * 1000):  # Wait for the specified duration
        pass