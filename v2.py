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

colliders = {}

beanbagshot = 0

def calculateXangle(target_x, target_y):
    yaw_rad = math.atan2(target_y, target_x)
    return math.degrees(yaw_rad)

def calculateYangle(target_x, target_y, target_z, v0, g=9.81):
    d = math.sqrt(target_x**2 + target_y**2)
    a = g * (d ** 2)
    b = -2 * (v0 ** 2) * d
    c = (2 * (v0 ** 2) * target_z) + (g * (d ** 2))
    
    discriminant = (b ** 2) - (4 * a * c)
    if discriminant < 0:
        return None
        
    tan_theta = (-b - math.sqrt(discriminant)) / (2 * a)
    launch_rad = math.atan(tan_theta)
    return math.degrees(launch_rad)


class Control:
    def __init__(self, bot, intake):
        self.controller = Controller()
        self.bot = bot
        self.intake = intake
        self.controller.buttonEUp.pressed(self.intake.run)



        def update(self):
        vAxis = self.controller.axisC.position()
        hAxis = self.controller.axisD.position()
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
        self.intakeMotor = Motor(Ports.PORT3, False)
        self.intakeMotor1 = Motor(Ports.PORT4, False)
        self.flywheelMotor1 = Motor(Ports.PORT5, False)
        self.flywheelMotor2 = Motor(Ports.PORT6, False)
        self.flywheelMotor1.set_velocity(100,PERCENT)
        self.flywheelMotor2.set_velocity(100,PERCENT)
        self.intakeMotor.set_velocity(85, PERCENT)
        self.

    def spinFlywheel(self):
        self.flywheelMotor1.spin(REVERSE)
        self.flywheelMotor2.spin(REVERSE)
    
    def intake(self):
        self.intakeMotor.spin(REVERSE)
    
    def run(self):
        global beanbagshot
        beanbagshot += 1
        self.spinFlywheel()
        self.intake()
        wait(25,SECONDS)
        

class Robot:
    def __init__(self, brain, inertial):
        self.brain = brain
        self.inertial = inertial
        self.x = 0
        self.y = 0
        self.mode = "controller"
        self.get = "loader"
        self.drivebase = Drivebase
        self.controller = Control(self.drivebase)
        self.moveTarget = ()
        self.aimTarget = None
    
    
    def odometry(self):
        pass
    
    def closestBeanbagPos(self):
        closestBeanBagPos = None
        i = 0
        for value in colliders["bluebean"]:
            x,y = value
            diffx, diffy = (abs(x - value[0]), abs(y - value[0]))

            if closestBeanBagPos == None:
                closestBeanBagPos = (diffx, diffy)
                   continue

            pytDiff = math.sqrt(diffx^2 + diffy^2)
            ollpyt = math.sqrt(closestBeanBagPos[0]^2+closestBeanBagPos[1]^2)
            
            if pytDiff < ollpyt:
                closestBeanBagPos = value
            i += 1

        return closestBeanBagPos+(i)
    
    def removeBeanbag(self):
        pass

    def find(self):
        if self.moveTarget == ():
            self.moveTarget = closestBeanbagPos
        
        if abs(self.x-self.moveTarget[0]) >= 3 and abs(self.x-self.moveTarget[1]) == 3:
            pass            



    def auto(self):
        if self.mode == "search":
            pass
        
        if self.mode == "loader":
            pass
    
    
    def update(self):
        self.odometry()
        if self.mode == "controller":
            self.controller.update()
        
        elif self.mode == "driver":
            self.auto()



robot = Robot()
while True:
    robot.update(brain, inertial)
    
    
