
from vex import *
import urandom
import math

brain=Brain()

# Ports to tune on the real robot.
LEFT_DRIVE_PORT = Ports.PORT1
RIGHT_DRIVE_PORT = Ports.PORT2
CLAW_ROTATE_PORT = Ports.PORT3
SLINGSHOT_PULL_PORT = Ports.PORT4
SLINGSHOT_RELEASE_PORT = Ports.PORT5
PNEUMATIC_PORT = Ports.PORT6

LEFT_DRIVE_REVERSED = False
RIGHT_DRIVE_REVERSED = True
CLAW_ROTATE_REVERSED = False
SLINGSHOT_PULL_REVERSED = False
SLINGSHOT_RELEASE_REVERSED = False

# Field/launcher numbers are placeholders. Calibrate these on the real field.
TARGET_X = 1800
TARGET_Y = 0
TARGET_Z = 500
LAUNCHER_Z = 140
DEFAULT_PITCH_DEGREES = 38
MIN_PULLBACK_DEGREES = 160
MAX_PULLBACK_DEGREES = 760
HEIGHT_TO_DISTANCE_GAIN = 0.45

# Each row is (effective horizontal distance in mm, pullback motor degrees).
# Replace these with the smallest pullback values that score reliably.
SHOT_TABLE = [
    (450, 180),
    (700, 260),
    (950, 350),
    (1200, 460),
    (1450, 585),
    (1700, 720),
]


brain = Brain()
brain_inertial = Inertial()
controller = Controller()


def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime))


initializeRandomSeed()


colliders = {}
colliders = {
    "bluebean": [],
}

beanbagshot = 0


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def calculateXangle(target_x, target_y):
    yaw_rad = math.atan2(target_y, target_x)
    return math.degrees(yaw_rad)


def calculateYangle(target_x, target_y, target_z, v0, g=9.81):
    d = math.sqrt(target_x**2 + target_y**2)
    a = g * (d ** 2)
    b = -2 * (v0 ** 2) * d
    c = (2 * (v0 ** 2) * target_z) + (g * (d ** 2))
    
    discriminant = (b ** 2) - (4 * a * c)
    d = math.sqrt(target_x ** 2 + target_y ** 2) / 1000.0
    height = target_z / 1000.0
    if d <= 0:
        return 90 if height > 0 else 0

    discriminant = v0 ** 4 - g * (g * d ** 2 + 2 * height * v0 ** 2)
    if discriminant < 0:
        return None
        
    tan_theta = (-b - math.sqrt(discriminant)) / (2 * a)

    tan_theta = (v0 ** 2 - math.sqrt(discriminant)) / (g * d)
    launch_rad = math.atan(tan_theta)
    return math.degrees(launch_rad)


def interpolate_pullback(effective_distance_mm):
    if effective_distance_mm <= SHOT_TABLE[0][0]:
        return SHOT_TABLE[0][1]

    for i in range(1, len(SHOT_TABLE)):
        d0, p0 = SHOT_TABLE[i - 1]
        d1, p1 = SHOT_TABLE[i]

        if effective_distance_mm <= d1:
            percent = (effective_distance_mm - d0) / (d1 - d0)
            return p0 + ((p1 - p0) * percent)

    return SHOT_TABLE[-1][1]


def calculate_pullback(distance_mm, height_change_mm=0):
    effective_distance = distance_mm + (height_change_mm * HEIGHT_TO_DISTANCE_GAIN)
    raw_pullback = interpolate_pullback(effective_distance)
    return clamp(raw_pullback, MIN_PULLBACK_DEGREES, MAX_PULLBACK_DEGREES)


def calculate_autoaim(robot_x, robot_y, robot_heading, target_x, target_y, target_z):
    dx = target_x - robot_x
    dy = target_y - robot_y
    dz = target_z - LAUNCHER_Z
    distance_mm = math.sqrt(dx ** 2 + dy ** 2)
    world_yaw = math.degrees(math.atan2(dy, dx))
    yaw_error = normalize_angle(world_yaw - robot_heading)
    effective_distance = distance_mm + (dz * HEIGHT_TO_DISTANCE_GAIN)
    raw_pullback = interpolate_pullback(effective_distance)
    pullback = clamp(raw_pullback, MIN_PULLBACK_DEGREES, MAX_PULLBACK_DEGREES)

    return {
        "yaw": yaw_error,
        "pitch": DEFAULT_PITCH_DEGREES,
        "pullback": pullback,
        "distance": distance_mm,
        "effective_distance": effective_distance,
        "reachable": raw_pullback >= MIN_PULLBACK_DEGREES and raw_pullback <= MAX_PULLBACK_DEGREES,
    }


class Control:
    def __init__(self, bot):
        self.controller = Controller()
    def __init__(self, bot, vex_controller):
        self.controller = vex_controller
        self.bot = bot
        # self.controller.buttonEUp.pressed(self.intake.run)
        self.deadband = 8
        self.last_fire_pressed = False
        self.last_load_pressed = False

    def apply_deadband(self, value):
        if abs(value) < self.deadband:
            return 0
        return value

    def update(self):
        pass
        
            
        if self.controller.buttonRDown.pressing():
            self.bot.autoaim()
        else:
            left_speed = self.apply_deadband(self.controller.axisA.position())
            right_speed = self.apply_deadband(self.controller.axisD.position())
            self.bot.drivebase.tank_drive(left_speed, right_speed)

        if self.controller.buttonEUp.pressing():
            self.bot.claw.open()
        elif self.controller.buttonEDown.pressing():
            self.bot.claw.close()

        if self.controller.buttonFUp.pressing():
            self.bot.claw.rotate_up()
        elif self.controller.buttonFDown.pressing():
            self.bot.claw.rotate_down()
        else:
            self.bot.claw.stop_rotate()

        load_pressed = self.controller.buttonLUp.pressing()
        if load_pressed and not self.last_load_pressed:
            self.bot.slingshot.load_to_pullback(self.bot.last_aim["pullback"])
        self.last_load_pressed = load_pressed

        fire_pressed = self.controller.buttonRUp.pressing()
        if fire_pressed and not self.last_fire_pressed:
            self.bot.slingshot.fire()
        self.last_fire_pressed = fire_pressed


class Drivebase:
    def __init__(self):
        self.motor1 = Motor(Ports.PORT1, False)
        self.motor2 = Motor(Ports.PORT2, False)
    
        self.left_motor = Motor(LEFT_DRIVE_PORT, LEFT_DRIVE_REVERSED)
        self.right_motor = Motor(RIGHT_DRIVE_PORT, RIGHT_DRIVE_REVERSED)
        self.left_motor.set_stopping(BRAKE)
        self.right_motor.set_stopping(BRAKE)

    def reset(self):
        self.left_motor.reset_position()
        self.right_motor.reset_position()
        self.stop()

    def left_encoding(self):
        return self.left_motor.position(DEGREES)

    def right_encoding(self):
        return self.right_motor.position(DEGREES)

    def spin_signed(self, motor, speed):
        speed = clamp(speed, -100, 100)
        if speed == 0:
            motor.stop()
        elif speed > 0:
            motor.spin(FORWARD, speed, PERCENT)
        else:
            motor.spin(REVERSE, abs(speed), PERCENT)

    def tank_drive(self, left_speed, right_speed):
        self.spin_signed(self.left_motor, left_speed)
        self.spin_signed(self.right_motor, right_speed)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def forward(self, speed=60):
        self.tank_drive(speed, speed)

    def backward(self, speed=60):
        self.tank_drive(-speed, -speed)

    def right(self, speed=45):
        self.tank_drive(speed, -speed)

    def left(self, speed=45):
        self.tank_drive(-speed, speed)


class Claw:
    def __init__(self):
        self.rotate_motor = Motor(CLAW_ROTATE_PORT, CLAW_ROTATE_REVERSED)
        self.pneumatic = Pneumatic(PNEUMATIC_PORT)
        self.rotate_motor.set_stopping(HOLD)

    def open(self):
        self.pneumatic.extend(CYLINDERALL)

    def close(self):
        self.pneumatic.retract(CYLINDERALL)

    def rotate_up(self):
        self.rotate_motor.spin(FORWARD, 55, PERCENT)

    def rotate_down(self):
        self.rotate_motor.spin(REVERSE, 55, PERCENT)

    def stop_rotate(self):
        self.rotate_motor.stop(HOLD)


class LinearSlingshot:
    def __init__(self):
        self.pull_motor = Motor(SLINGSHOT_PULL_PORT, SLINGSHOT_PULL_REVERSED)
        self.release_motor = Motor(SLINGSHOT_RELEASE_PORT, SLINGSHOT_RELEASE_REVERSED)
        self.pull_motor.set_stopping(HOLD)
        self.release_motor.set_stopping(BRAKE)
        self.target_pullback = MIN_PULLBACK_DEGREES

    def reset(self):
        self.motor1.set_velocity(90,PERCENT)
        self.motor2.set_velocity(90,PERCENT)
        self.pull_motor.reset_position()
        self.release_motor.reset_position()
        self.pull_motor.stop(HOLD)
        self.release_motor.stop()

    def load_to_pullback(self, pullback_degrees):
        self.target_pullback = clamp(
            pullback_degrees,
            MIN_PULLBACK_DEGREES,
            MAX_PULLBACK_DEGREES,
        )
        self.pull_motor.spin_to_position(
            self.target_pullback,
            DEGREES,
            70,
            PERCENT,
            False,
        )

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
        self.motor1.set_velocity(90, PERCENT)
        self.motor2.set_velocity(50, PERCENT)
        self.motor1.spin(FORWARD)
        self.motor2.spin(REVERSE)
    
    def forwardRight(self):
        self.motor1.set_velocity(50, PERCENT)
        self.motor2.set_velocity(90, PERCENT)
        self.motor1.spin(REVERSE)
        self.motor2.spin(FORWARD)
        
    def load_for_distance(self, distance_mm):
        self.load_to_pullback(calculate_pullback(distance_mm))

    def fire(self):
        global beanbagshot
        beanbagshot += 1
        self.release_motor.spin_for(FORWARD, 90, DEGREES, 80, PERCENT)
        wait(150, MSEC)
        self.release_motor.spin_for(REVERSE, 90, DEGREES, 80, PERCENT)
        self.pull_motor.reset_position()

        
class 

class Robot:
    def __init__(self, brain, inertial):
        self.brain = brain
    def __init__(self, robot_brain, inertial, vex_controller):
        self.brain = robot_brain
        self.inertial = inertial
        self.x = 0
        self.y = 0
        self.last_left_position = 0
        self.last_right_position = 0
        self.mode = "controller"
        self.get = "loader"
        self.drivebase = Drivebase
        self.controller = Control(self.drivebase)
        self.drivebase = Drivebase()
        self.claw = Claw()
        self.slingshot = LinearSlingshot()
        self.controller = Control(self, vex_controller)
        self.moveTarget = ()
        self.aimTarget = None
    
    
        self.aimTarget = (TARGET_X, TARGET_Y, TARGET_Z)
        self.last_aim = calculate_autoaim(
            self.x,
            self.y,
            self.heading(),
            TARGET_X,
            TARGET_Y,
            TARGET_Z,
        )

    def setup(self):
        self.inertial.calibrate()
        while self.inertial.is_calibrating():
            wait(50, MSEC)
        self.inertial.reset_heading()
        self.drivebase.reset()
        self.slingshot.reset()

    def heading(self):
        return self.inertial.heading(DEGREES)

    def odometry(self):
        pass
    
        left_position = self.drivebase.left_encoding()
        right_position = self.drivebase.right_encoding()
        self.last_left_position = left_position
        self.last_right_position = right_position

    def closestBeanbagPos(self):
        closestBeanBagPos = None
        closestIndex = None
        closestDistance = None
        i = 0
        for value in colliders["bluebean"]:
            x,y = value
            diffx, diffy = (abs(x - value[0]), abs(y - value[0]))

            if closestBeanBagPos == None:
                closestBeanBagPos = (diffx, diffy)
                continue
        for value in colliders.get("bluebean", []):
            x, y = value
            diffx = x - self.x
            diffy = y - self.y
            distance = math.sqrt(diffx ** 2 + diffy ** 2)

            pytDiff = math.sqrt(diffx^2 + diffy^2)
            ollpyt = math.sqrt(closestBeanBagPos[0]^2+closestBeanBagPos[1]^2)
            
            if pytDiff < ollpyt:
            if closestDistance is None or distance < closestDistance:
                closestDistance = distance
                closestBeanBagPos = value
                closestIndex = i

            i += 1

        return closestBeanBagPos+(i)
    
    def removeBeanbag(self):
        pass
        if closestBeanBagPos is None:
            return None

        return closestBeanBagPos + (closestIndex,)

    def removeBeanbag(self, index):
        beanbags = colliders.get("bluebean", [])
        if index is not None and index >= 0 and index < len(beanbags):
            beanbags.pop(index)

    def find(self):
        if self.moveTarget == ():
            self.moveTarget = self.closestBeanbagPos()
        
        if abs(self.x-self.moveTarget[0]) >= 3 and abs(self.x-self.moveTarget[1]) == 3:
            pass            

        if self.moveTarget is None:
            self.drivebase.stop()
            return

        target_x = self.moveTarget[0]
        target_y = self.moveTarget[1]
        dx = target_x - self.x
        dy = target_y - self.y

        if abs(dx) <= 30 and abs(dy) <= 30:
            self.drivebase.stop()
            return

        target_heading = math.degrees(math.atan2(dy, dx))
        error = normalize_angle(target_heading - self.heading())

        if abs(error) > 8:
            if error > 0:
                self.drivebase.right(35)
            else:
                self.drivebase.left(35)
        else:
            self.drivebase.forward(45)

    def autoaim(self):
        if self.aimTarget is None:
            self.aimTarget = (TARGET_X, TARGET_Y, TARGET_Z)

        target_x, target_y, target_z = self.aimTarget
        self.last_aim = calculate_autoaim(
            self.x,
            self.y,
            self.heading(),
            target_x,
            target_y,
            target_z,
        )

        if self.last_aim["reachable"]:
            self.slingshot.load_to_pullback(self.last_aim["pullback"])

        if abs(self.last_aim["yaw"]) > 4:
            if self.last_aim["yaw"] > 0:
                self.drivebase.right(28)
            else:
                self.drivebase.left(28)
        else:
            self.drivebase.stop()

        return self.last_aim

    def auto(self):
        if self.mode == "search":
            pass
        
        if self.mode == "loader":
            pass
    
    
            self.find()
        elif self.mode == "loader":
            self.autoaim()
        else:
            self.drivebase.stop()

    def update(self):
        self.odometry()
        if self.mode == "controller":
            self.controller.update()
        
        elif self.mode == "driver":
            self.auto()
        else:
            self.drivebase.stop()


robot = Robot(brain, brain_inertial, controller)
robot.setup()

robot = Robot(brain, brain_inertial)
while True:
    robot.update()
    
    
    wait(20, MSEC)
