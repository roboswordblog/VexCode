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

#dictionary for x's and y's
colliders = {}

#math


#classes for bot

class Remote:
    def __init__(self):
        pass

class Drivebase:
    def __init__(self):
        self.motor1 = Motor(Ports.PORT1, False)
        self.motor2 = Motor(Ports.PORT2, False)
    
    def motor1Encodding(self):
        return self.motor1.position(DEGREES)
    
    def motor2Encodding(self):
        return self.motor2.position(DEGREES)
    
    def forward(self):
        self.motor1.spin(FORWARD)
        self.motor2.spin(FORWARD)
    
class IntakeFlywheel:
    def __init__(self):
        pass

class Robot:
    def __init__(self, brain, inertial):
        self.brain = brain
        self.inertial = inertial
        self.x = 0
        self.y = 0
    
    def odometry(self):
        pass
    
    
