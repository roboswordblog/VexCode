from vex import *
import urandom
import math

brain=Brain()

class Control:
    def __init__(self, bot):
        self.controller = Controller()
        self.bot = bot
    
    def update(self):
        jA = self.controller.axisA.position()


class Bot:
    def __init__(self):
        # motors
        self.driveMotors = [Motor(Ports.PORT1, False), Motor(Ports.PORT2, False)]
        self.intakeMotor = Motor(Ports.PORT3, False)
        self.cannonRotateMotors = None
        self.cannonMotors = None # not designed yet
        # sensor
        self.colorsensor = None
        self.distanceSensorOne = None
        self.distanceSensorTwo = None
        self.inertiaSensor = Inertial()
    
    def forward(self):
        self.driveMotors[0].spin(FORWARD)
        self.driveMotors[1].spin(FORWARD)
    
    def backward(self):
        self.driveMotors[0].spin(REVERSE)
        self.driveMotors[1].spin(REVERSE)
    
    def right(self):
        self.driveMotors[0].spin(FORWARD)
        self.driveMotors[1].spin(REVERSE)

    def left(self):
        self.driveMotors[0].spin(REVERSE)
        self.driveMotors[1].spin(FORWARD)

    def boost(self):
        motor_1.set_velocity(20, PERCENT)


    def intake(self):
        pass
    
    def autoMove(self):
        pass
    
    def cannonRotate(self):
        pass


