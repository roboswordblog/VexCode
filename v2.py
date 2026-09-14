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
class Control:
    def __init__(self, bot):
        self.controller = Controller()
        self.bot = bot


        def update(self):
        vAxis = self.controller.axisA.position()
        hAxis = self.controller.axisB.position()
        DEADBAND = 10
    
        if vAxis > DEADBAND and hAxis > DEADBAND:
            self.bot.forwardRight()
            
        elif vAxis > DEADBAND and hAxis < -DEADBAND:
            self.bot.forwardLeft()
            
        elif vAxis < -DEADBAND and hAxis > DEADBAND:
            self.bot.backwardRight()
            
        elif vAxis < -DEADBAND and hAxis < -DEADBAND:
            self.bot.backwardLeft()
            
        elif vAxis > DEADBAND:
            self.bot.forward()
            
        elif vAxis < -DEADBAND:
            self.bot.backward()
            
        elif hAxis > DEADBAND:
            self.bot.right()
            
        elif hAxis < -DEADBAND:
            self.bot.left()
        
        else:
            self.bot.stop()
            

class Drivebase:
    def __init__(self):
        self.motor1 = Motor(Ports.PORT1, False)
        self.motor2 = Motor(Ports.PORT2, False)
    
    def reset(self):
        self.motor1.set_velocity(90,PERCENT)
        self.motor2.set_velocity(90,PERCENT)

    def motor1Encodding(self):
        return self.motor1.position(DEGREES)
    
    def motor2Encodding(self):
        return self.motor2.position(DEGREES)
    
    def forward(self):
        self.motor1.spin(FORWARD)
        self.motor2.spin(FORWARD)
    
    def backward(self):
        self.motor1.spin(REVERSE)
        self.motor2.spin(REVERSE)
    
    def right(self):
        self.motor1.spin(REVERSE)
        self.motor2.spin(FORWARD)
    
    def left(self):
        self.motor1.spin(FORWARD)
        self.motor2.spin(REVERSE)
    
    def forwardLeft(self):
        self.motor1.speed(90)
        self.motor2.speed(50)
        self.motor1.spin(FORWARD)
        self.motor2.spin(REVERSE)
    
    def forwardRight(self):
        self.motor1.speed(50)
        self.motor2.speed(90)
        self.motor1.spin(REVERSE)
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
        self.mode = "controller"
        self.drivebase = Drivebase
        self.controller = Control(self.drivebase)
        
    
    def odometry(self):
        pass
    
    def update(self):
        if self.mode == "controller":
            self.controller.update()

robot = Robot()
while True:
    robot.update()
    
    
