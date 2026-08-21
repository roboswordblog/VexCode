from vex import *
import urandom
import math

brain=Brain()

def check_collision(x1, y1, width1, length1, x2, y2, width2, length2):
    left1 = x1
    right1 = x1 + width1
    bottom1 = y1
    top1 = y1 + length1

    left2 = x2
    right2 = x2 + width2
    bottom2 = y2
    top2 = y2 + length2

    if (right1 <= left2 or 
        left1 >= right2 or 
        top1 <= bottom2 or 
        bottom1 >= top2):
        return False
        
    return True


beanBagList = []
class BeanBag:
    def __init__(self, x, y, width, height):
        beanBagList.append(self)
        self.x = x
        self.y = y
        self.width = width
        self.height = height

class Control:
    def __init__(self, bot):
        self.controller = Controller()
        self.bot = bot
        self.controller.buttonEUp.pressed(self.bot.boost)
        self.controller.buttonEUp.pressed(self.bot.boost)
        self.controller.buttonEUp.pressed(self.bot.boost)
    

    def update(self):
        vAxis = self.controller.axisA.position()
        hAxis = self.controller.axisB.position()
        
        if self.vAxis > 10:
            self.bot.forward()
        elif self.vAxis < -10:
            self.bot.backward()
        
        if self.hAxis > 10:
            self.bot.right()
        elif self.hAxis < -10:
            self.bot.left()
        


class Bot:
    def __init__(self):
        # used to calculate
        self.x = 0
        self.y = 0
        self.width = 0
        self.height = 0

        # motors
        self.driveMotors = [Motor(Ports.PORT1, False), Motor(Ports.PORT2, False)]
        self.intakeMotor = Motor(Ports.PORT3, False)
        self.cannonRotateMotors = [Motor(Ports.PORT4, False), Motor(Ports.PORT5, False)]
        self.cannonMotors = [Motor(Ports.PORT6, False), Motor(Ports.PORT7, False)]

        # sensor
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
        self.driveMotors[0].set_velocity(75, PERCENT)
        self.driveMotors[1].set_velocity(75, PERCENT)
        #self.cannonMotors.set_velocity(75, PERCENT)


    def intake(self):
        pass
    
    def autoMove(self):
        pass
    
    def cannonRotate(self):
        pass

    def update(self):
        pass
