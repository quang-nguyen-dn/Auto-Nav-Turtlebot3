import numpy as np
IS_AUTONOMOUS = False
X_TARGET = 2.0
Y_TARGET = 2.0
STOP_THRESHOLD = 0.03           # Unit: m

ROBOT_MARGIN = 130              # Unit: mm
THRESHOLD = 3.6                 # Unit: m
MIN_THRESHOLD = 0.1
THRESHOLD_STEP = 0.25
THRESHOLD_ANGLE = 95            # Unit: deg, has to be greater than 90 deg

ANGLE_TO_START_MOVING = 10 / 180*np.pi  # Unit: rad

class Colour:
    #OpenCV use BGR tuple
    def __init__(self):
        self.blue = (255, 0, 0)
        self.green = (0, 255, 0)
        self.light_green = (0, 255, 110)
        self.red = (0, 0, 255)
        self.yellow = (0, 220, 255)
        self.orange = (0, 120, 255)
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
    def grey(self, percentage):
        level = int(percentage/100*255)
        return (level, level, level)
    def Green(self, percentage=100):
        level = int(percentage/100*255)
        return (0, level, 0)