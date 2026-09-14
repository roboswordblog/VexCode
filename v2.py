from vex import *
import urandom
import math

brain=Brain()

brain_inertial = Inertial()

def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 
    
initializeRandomSeed()

from vex import *

#dictionary for x's and y's
colliders = {}

#math


#classes for bot

class Remote:
    def __init__(self):
        pass

class Drivebase:
    def __init__(self):
        pass

class IntakeFlywheel:
    def __init__(self):
        pass

class Robot:
    def __init__(self):
        pass
    
    def odometry(self):
        pass
    
    
