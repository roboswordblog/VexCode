from vex import *
import urandom
import math

brain=Brain()

def normalize_angle(angle):
    return (angle + 180) % 360 - 180


def calculate_aim(
    robot_x, robot_y, robot_z,
    target_x, target_y, target_z,
    robot_heading=0,
    launch_speed=4.31,
    min_pitch=-10,
    max_pitch=80,
    high_arc=False
):
    g = 9.81

    dx = target_x - robot_x
    dy = target_y - robot_y
    dz = target_z - robot_z

    horizontal_distance = math.sqrt(dx**2 + dy**2)
    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    world_yaw = math.degrees(math.atan2(dy, dx))
    yaw = normalize_angle(world_yaw - robot_heading)

    if horizontal_distance < 0.000001:
        if dz > 0:
            pitch = 90.0
        elif dz < 0:
            pitch = -90.0
        else:
            pitch = 0.0

        if pitch < min_pitch or pitch > max_pitch:
            return {
                "yaw": round(yaw, 2),
                "pitch": None,
                "flywheel_power": 100,
                "distance": round(distance, 3),
                "reachable": False,
                "reason": "Target requires pitch outside launcher limits."
            }

        return {
            "yaw": round(yaw, 2),
            "pitch": round(pitch, 2),
            "flywheel_power": 100,
            "distance": round(distance, 3),
            "reachable": True,
            "reason": "Target is directly above/below launcher."
        }

    v = launch_speed
    R = horizontal_distance
    H = dz

    discriminant = v**4 - g * (g * R**2 + 2 * H * v**2)

    if discriminant < 0:
        return {
            "yaw": round(yaw, 2),
            "pitch": None,
            "flywheel_power": 100,
            "distance": round(distance, 3),
            "reachable": False,
            "reason": "Target is out of range at 100% flywheel."
        }

    sqrt_discriminant = math.sqrt(discriminant)

    numerator_low = v**2 - sqrt_discriminant
    numerator_high = v**2 + sqrt_discriminant
    denominator = g * R

    low_angle = math.degrees(
        math.atan(numerator_low / denominator)
    )

    high_angle = math.degrees(
        math.atan(numerator_high / denominator)
    )

    pitch = high_angle if high_arc else low_angle

    if pitch < min_pitch or pitch > max_pitch:
        other_pitch = low_angle if high_arc else high_angle

        if min_pitch <= other_pitch <= max_pitch:
            pitch = other_pitch
        else:
            return {
                "yaw": round(yaw, 2),
                "pitch": None,
                "flywheel_power": 100,
                "distance": round(distance, 3),
                "reachable": False,
                "reason": "Both ballistic trajectories are outside the pitch limits.",
                "low_pitch": round(low_angle, 2),
                "high_pitch": round(high_angle, 2)
            }

    return {
        "yaw": round(yaw, 2),
        "pitch": round(pitch, 2),
        "flywheel_power": 100,
        "distance": round(distance, 3),
        "reachable": True,
        "reason": "Target is reachable.",
        "low_pitch": round(low_angle, 2),
        "high_pitch": round(high_angle, 2)
    }

    
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

# questions : why should i use sensors in vex 

beanBagList = []
class BeanBag:
    def __init__(self, x, y):
        beanBagList.append(self)
        self.x = x
        self.y = y
        self.width = width
        self.height = height

class Level:
    def __init__(self, x, y, width, height, level):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.points = level
        self.level = level

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
        self.chainMotor =  Motor(Ports.PORT8, False)
        self.cannonRotateMotors = [Motor(Ports.PORT4, False), Motor(Ports.PORT5, False)]
        self.cannonMotors = [Motor(Ports.PORT6, False), Motor(Ports.PORT7, False)]

        # sensor
        self.inertiaSensor = Inertial()
        self.colorSensor = Optical(Ports.PORT9) 
    
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
        self.driveMotors[0].set_velocity(100, PERCENT)
        self.driveMotors[1].set_velocity(100, PERCENT)

    def intake(self):
        self.intakeMotor.set_velocity(75, PERCENT)
        self.intakeMotor.spin(FORWARD)
    
    def autoMove(self):
        pass
    
    def cannonRotate(self):
        pass

    def update(self):
        pass

