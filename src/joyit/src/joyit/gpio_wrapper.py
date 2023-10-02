try:
    from RPi.GPIO import *
except:
    from Mock.GPIO import *

def DigitalRead(pin):
  return 1
