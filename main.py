from vex import *
import urandom
import math

brain=Brain()

class Control:
    def __init__(self):
        self.controller = Controller()

class Bot:
    def __init__(self):
        # motors
        self.driveMotors = [Motor(Ports.PORT1, False), Motor(Ports.PORT2, False)]
        self.intakeMotor = Motor(Ports.PORT3, False)
        self.cannonMotors = None # not designed yet
        # sensor
        self.colorsensor = None
        self.distanceSensorOne = None
        self.distanceSensorTwo = None
        self.inertiaSensor = Inertial()
    
    def forward(self):
        pass
    
    def backward(self):
        pass
    
    def right(self):
        pass
    
    def boost(self):
        pass

    def intake(self):
        pass
    
    def move(self):
        pass



