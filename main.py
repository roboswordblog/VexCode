from vex import *
import urandom
import math

def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 


initializeRandomSeed()


brain=Brain()
import math


# ============================================================
# AIM / BALLISTIC SOLVER
# ============================================================
#
# Main function:
#
# calculate_aim(...)
#
# The solver:
#   1. Calculates the horizontal direction to the target.
#   2. Tries every flywheel percentage in the requested range.
#   3. Converts motor RPM -> flywheel RPM using the gear ratios.
#   4. Converts flywheel RPM -> beanbag launch velocity.
#   5. Tries launch angles.
#   6. Simulates the beanbag through the air.
#   7. Includes gravity and quadratic air resistance.
#   8. Checks the entire trajectory against every obstacle.
#   9. Checks whether the beanbag reaches the target.
#  10. Returns the best valid shot.
#
# IMPORTANT:
# The motor RPM -> launch velocity relationship is NOT something
# physics alone can determine. Therefore:
#
#     velocity_efficiency
#
# is supplied as an argument and should be calibrated on your robot.
#
# ============================================================


def normalize_angle(angle):
    return (angle + 180) % 360 - 180


def _point_inside_box(x, y, z, box):
    """
    Check whether a point is inside a 3D rectangular obstacle.

    box must contain:
        xmin
        xmax
        ymin
        ymax
        zmin
        zmax
    """

    return (
        box["xmin"] <= x <= box["xmax"] and
        box["ymin"] <= y <= box["ymax"] and
        box["zmin"] <= z <= box["zmax"]
    )


def _distance_3d(x1, y1, z1, x2, y2, z2):
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1

    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _distance_to_goal(
    x,
    y,
    z,
    target_x,
    target_y,
    target_z
):
    return _distance_3d(
        x,
        y,
        z,
        target_x,
        target_y,
        target_z
    )


def _simulate_trajectory(
    start_x,
    start_y,
    start_z,

    yaw_degrees,
    pitch_degrees,

    launch_velocity,

    gravity,
    air_density,
    drag_coefficient,
    cross_sectional_area,
    beanbag_mass,

    wind_x,
    wind_y,
    wind_z,

    time_step,
    max_time,

    target_x,
    target_y,
    target_z,
    goal_tolerance,

    obstacles
):
    """
    Numerically simulates the beanbag.

    Returns a dictionary describing what happened.

    Drag model:

        F_drag = 0.5 * rho * Cd * A * v^2

    The drag force points opposite the beanbag's velocity
    relative to the air.
    """

    yaw = math.radians(yaw_degrees)
    pitch = math.radians(pitch_degrees)

    # --------------------------------------------------------
    # Initial velocity
    # --------------------------------------------------------

    horizontal_velocity = (
        launch_velocity * math.cos(pitch)
    )

    vx = horizontal_velocity * math.cos(yaw)
    vy = horizontal_velocity * math.sin(yaw)
    vz = launch_velocity * math.sin(pitch)

    x = start_x
    y = start_y
    z = start_z

    time = 0.0

    maximum_height = z

    minimum_goal_distance = float("inf")

    collided = False
    reached_goal = False

    collision_obstacle = None

    trajectory = []

    while time <= max_time:

        # ----------------------------------------------------
        # Store current point
        # ----------------------------------------------------

        trajectory.append((x, y, z))

        # ----------------------------------------------------
        # Update maximum height
        # ----------------------------------------------------

        if z > maximum_height:
            maximum_height = z

        # ----------------------------------------------------
        # Check obstacle collision
        # ----------------------------------------------------

        for obstacle in obstacles:

            if _point_inside_box(
                x,
                y,
                z,
                obstacle
            ):
                collided = True
                collision_obstacle = obstacle

                return {
                    "reached_goal": False,
                    "collided": True,
                    "collision_obstacle": collision_obstacle,
                    "flight_time": time,
                    "maximum_height": maximum_height,
                    "minimum_goal_distance": minimum_goal_distance,
                    "trajectory": trajectory
                }

        # ----------------------------------------------------
        # Check target distance
        # ----------------------------------------------------

        goal_distance = _distance_to_goal(
            x,
            y,
            z,
            target_x,
            target_y,
            target_z
        )

        if goal_distance < minimum_goal_distance:
            minimum_goal_distance = goal_distance

        if goal_distance <= goal_tolerance:

            reached_goal = True

            return {
                "reached_goal": True,
                "collided": False,
                "collision_obstacle": None,
                "flight_time": time,
                "maximum_height": maximum_height,
                "minimum_goal_distance": minimum_goal_distance,
                "trajectory": trajectory
            }

        # ----------------------------------------------------
        # Beanbag hit the ground
        # ----------------------------------------------------

        if z < 0:
            return {
                "reached_goal": False,
                "collided": False,
                "collision_obstacle": None,
                "flight_time": time,
                "maximum_height": maximum_height,
                "minimum_goal_distance": minimum_goal_distance,
                "trajectory": trajectory
            }

        # ----------------------------------------------------
        # Velocity relative to air
        #
        # Wind is velocity of the air itself.
        # ----------------------------------------------------

        relative_vx = vx - wind_x
        relative_vy = vy - wind_y
        relative_vz = vz - wind_z

        relative_speed = math.sqrt(
            relative_vx * relative_vx +
            relative_vy * relative_vy +
            relative_vz * relative_vz
        )

        # ----------------------------------------------------
        # Calculate drag
        # ----------------------------------------------------

        if relative_speed > 0:

            drag_force = (
                0.5 *
                air_density *
                drag_coefficient *
                cross_sectional_area *
                relative_speed *
                relative_speed
            )

            # Drag acceleration
            drag_acceleration = (
                drag_force / beanbag_mass
            )

            ax_drag = (
                -drag_acceleration *
                relative_vx /
                relative_speed
            )

            ay_drag = (
                -drag_acceleration *
                relative_vy /
                relative_speed
            )

            az_drag = (
                -drag_acceleration *
                relative_vz /
                relative_speed
            )

        else:

            ax_drag = 0
            ay_drag = 0
            az_drag = 0

        # ----------------------------------------------------
        # Total acceleration
        # ----------------------------------------------------

        ax = ax_drag
        ay = ay_drag
        az = az_drag - gravity

        # ----------------------------------------------------
        # Numerical integration
        #
        # This uses simple Euler integration.
        # ----------------------------------------------------

        vx = vx + ax * time_step
        vy = vy + ay * time_step
        vz = vz + az * time_step

        x = x + vx * time_step
        y = y + vy * time_step
        z = z + vz * time_step

        time = time + time_step

    # --------------------------------------------------------
    # Maximum simulation time reached
    # --------------------------------------------------------

    return {
        "reached_goal": False,
        "collided": collided,
        "collision_obstacle": collision_obstacle,
        "flight_time": time,
        "maximum_height": maximum_height,
        "minimum_goal_distance": minimum_goal_distance,
        "trajectory": trajectory
    }


def calculate_aim(

    # ========================================================
    # ROBOT POSITION
    # ========================================================

    robot_x,
    robot_y,
    robot_z,

    robot_heading,

    # ========================================================
    # TARGET POSITION
    # ========================================================

    target_x,
    target_y,
    target_z,

    # ========================================================
    # MOTOR / FLYWHEEL
    # ========================================================

    motor_max_rpm,

    min_flywheel_percent=30,
    max_flywheel_percent=100,
    flywheel_percent_step=1,

    # Your gear system:
    #
    # big blue -> small blue
    # repeated 3 times
    #
    # Default:
    #
    # 60 / 12
    # 60 / 12
    # 60 / 12
    #
    # = 125x
    #
    gear_ratios=None,

    # Fraction of theoretical flywheel energy/speed
    # that actually becomes beanbag launch speed.
    #
    # THIS MUST BE CALIBRATED.
    #
    # This is deliberately an argument.
    velocity_efficiency=0.20,

    # ========================================================
    # LAUNCHER
    # ========================================================

    launcher_height=0.0,

    min_pitch=-10,
    max_pitch=80,
    pitch_step=1,

    # ========================================================
    # BEANBAG PHYSICS
    # ========================================================

    beanbag_mass=0.025,

    beanbag_diameter=0.08,

    drag_coefficient=0.47,

    air_density=1.225,

    # ========================================================
    # ENVIRONMENT
    # ========================================================

    gravity=9.81,

    wind_x=0.0,
    wind_y=0.0,
    wind_z=0.0,

    # ========================================================
    # SIMULATION
    # ========================================================

    time_step=0.005,
    max_simulation_time=5.0,

    # ========================================================
    # TARGET
    # ========================================================

    goal_tolerance=0.05,

    # ========================================================
    # OBSTACLES
    #
    # Each obstacle should be a dictionary:
    #
    # {
    #     "xmin": ...,
    #     "xmax": ...,
    #     "ymin": ...,
    #     "ymax": ...,
    #     "zmin": ...,
    #     "zmax": ...
    # }
    #
    # ========================================================

    obstacles=None,

    # ========================================================
    # OPTIMIZATION
    #
    # "lowest_power"
    # "shortest_time"
    # "highest_clearance"
    #
    # ========================================================

    optimization="lowest_power"
):


    if gear_ratios is None:
        gear_ratios = [
            60.0 / 12.0,
            60.0 / 12.0,
            60.0 / 12.0
        ]

    if obstacles is None:
        obstacles = []

    if motor_max_rpm <= 0:
        return {
            "reachable": False,
            "reason": "motor_max_rpm must be greater than zero."
        }

    if beanbag_mass <= 0:
        return {
            "reachable": False,
            "reason": "beanbag_mass must be greater than zero."
        }

    if beanbag_diameter <= 0:
        return {
            "reachable": False,
            "reason": "beanbag_diameter must be greater than zero."
        }

    if time_step <= 0:
        return {
            "reachable": False,
            "reason": "time_step must be greater than zero."
        }

    if flywheel_percent_step <= 0:
        return {
            "reachable": False,
            "reason": "flywheel_percent_step must be greater than zero."
        }

    if pitch_step <= 0:
        return {
            "reachable": False,
            "reason": "pitch_step must be greater than zero."
        }

    launch_z = robot_z + launcher_height

    dx = target_x - robot_x
    dy = target_y - robot_y

    horizontal_distance = math.sqrt(
        dx * dx +
        dy * dy
    )

    distance = math.sqrt(
        dx * dx +
        dy * dy +
        (target_z - launch_z) *
        (target_z - launch_z)
    )

    world_yaw = math.degrees(
        math.atan2(dy, dx)
    )

    yaw = normalize_angle(
        world_yaw - robot_heading
    )

    radius = beanbag_diameter / 2.0

    cross_sectional_area = (
        math.pi *
        radius *
        radius
    )

    if horizontal_distance < 0.000001:

        if target_z > launch_z:
            pitch = 90.0
        elif target_z < launch_z:
            pitch = -90.0
        else:
            pitch = 0.0

        if pitch < min_pitch or pitch > max_pitch:

            return {
                "reachable": False,
                "yaw": round(yaw, 2),
                "pitch": None,
                "reason":
                    "Target requires a pitch outside launcher limits."
            }

    candidates = []

    flywheel_percent = min_flywheel_percent

    while flywheel_percent <= max_flywheel_percent:
        motor_rpm = (
            motor_max_rpm *
            (flywheel_percent / 100.0)
        )


        total_gear_ratio = 1.0

        for ratio in gear_ratios:
            total_gear_ratio *= ratio

        flywheel_rpm = (
            motor_rpm *
            total_gear_ratio
        )

        flywheel_angular_velocity = (
            flywheel_rpm *
            2.0 *
            math.pi /
            60.0
        )

        flywheel_radius = radius

        theoretical_exit_velocity = (
            flywheel_angular_velocity *
            flywheel_radius
        )

        launch_velocity = (
            theoretical_exit_velocity *
            velocity_efficiency
        )

        if launch_velocity <= 0:

            flywheel_percent += flywheel_percent_step
            continue

        pitch = min_pitch

        while pitch <= max_pitch:

            simulation = _simulate_trajectory(

                robot_x,
                robot_y,
                launch_z,

                yaw,
                pitch,

                launch_velocity,

                gravity,
                air_density,
                drag_coefficient,
                cross_sectional_area,
                beanbag_mass,

                wind_x,
                wind_y,
                wind_z,

                time_step,
                max_simulation_time,

                target_x,
                target_y,
                target_z,

                goal_tolerance,

                obstacles
            )

            # ------------------------------------------------
            # Valid shot?
            # ------------------------------------------------

            if (
                simulation["reached_goal"] and
                not simulation["collided"]
            ):


                clearance = (
                    simulation["maximum_height"] -
                    max(launch_z, target_z)
                )

                candidate = {
                    "yaw": round(yaw, 2),
                    "pitch": round(pitch, 2),

                    "flywheel_power":
                        round(flywheel_percent, 2),

                    "motor_rpm":
                        round(motor_rpm, 2),

                    "flywheel_rpm":
                        round(flywheel_rpm, 2),

                    "launch_velocity":
                        round(launch_velocity, 3),

                    "distance":
                        round(distance, 3),

                    "flight_time":
                        round(
                            simulation["flight_time"],
                            3
                        ),

                    "maximum_height":
                        round(
                            simulation["maximum_height"],
                            3
                        ),

                    "clearance":
                        round(clearance, 3),

                    "minimum_goal_distance":
                        round(
                            simulation[
                                "minimum_goal_distance"
                            ],
                            4
                        ),

                    "reachable": True,

                    "trajectory":
                        simulation["trajectory"]
                }

                candidates.append(candidate)

            pitch += pitch_step

        flywheel_percent += flywheel_percent_step

    if len(candidates) == 0:

        return {
            "reachable": False,

            "yaw": round(yaw, 2),

            "pitch": None,

            "flywheel_power": None,

            "distance": round(distance, 3),

            "reason":
                "No flywheel velocity and pitch combination "
                "could reach the target without colliding "
                "with an obstacle."
        }

    # ========================================================
    # CHOOSE BEST SHOT
    # ========================================================

    if optimization == "lowest_power":

        # Lowest flywheel power first.
        #
        # If equal power:
        # choose the lower pitch.
        #

        best = min(
            candidates,
            key=lambda shot: (
                shot["flywheel_power"],
                shot["pitch"]
            )
        )

    elif optimization == "shortest_time":

        best = min(
            candidates,
            key=lambda shot: (
                shot["flight_time"],
                shot["flywheel_power"]
            )
        )

    elif optimization == "highest_clearance":

        best = max(
            candidates,
            key=lambda shot: (
                shot["clearance"],
                -shot["flywheel_power"]
            )
        )

    else:

        # Default to lowest power if an invalid optimization
        # mode was supplied.

        best = min(
            candidates,
            key=lambda shot: (
                shot["flywheel_power"],
                shot["pitch"]
            )
        )

    # ========================================================
    # ADD SOME GENERAL INFORMATION
    # ========================================================

    best["reason"] = (
        "Found a valid trajectory that reaches the target "
        "without colliding with an obstacle."
    )

    best["number_of_valid_shots"] = len(candidates)

    best["horizontal_distance"] = round(
        horizontal_distance,
        3
    )

    best["total_distance"] = round(
        distance,
        3
    )

    best["gear_ratio"] = total_gear_ratio

    return best


beanBagList = []
class BeanBag:
    def __init__(self, x, y):
        beanBagList.append(self)
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    
# only real obsticle is level 4, so only that should be a collision object, and  that should also be included in the caalculation of the beanbag being shot.
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
        self.cannonRotateMotor = Ports.PORT4, False # other one will be the  car
        self.cannonMotors = [Motor(Ports.PORT6, False), Motor(Ports.PORT7, False)]

        # sensor
        self.inertiaSensor = Inertial()
        self.colorSensor = Optical(Ports.PORT9) 

        # stored movements
        self.cannonRotateY = 0

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


    # NOTE: FOR INTAKE AND OUTTAKE I STILL HAVENT MEASURED THE TIME IT WILL TAKE FOR THEM
    async def intake(self):
        self.intakeMotor.set_velocity(75, PERCENT)
        self.intakeMotor.spin(FORWARD)
        wait(4, SECONDS)

    async def outtake(self):
        self.intakeMotor.spin(REVERSE)
        wait(4, SECONDS)
    
    async def elevator(self):
        # self.elevatorMotor.

    def autoMove(self):
        pass
    
    def getAimParameters(self):
        x1 = self.x
        y1 = self.y
        z1 = self.z

    async def cannonRotate(self):
        pass

    def update(self):
        pass

brain.screen.print("THE CHEFS")

# compression is only needed on the back side, so with a flip and  compression over there we  can make it drive
# this is asumming that the bottom side of the gear isnt the culprite, which in that case we will need to put the other wheels and the smaller gear lower, and add the compressoin for the big gear.
# or, if that happens, we can just use a small piece and connect it with the compressions so it doesn't interfere with the wheels

bot = Bot()
controller = Bot()
mode = 1
# 1 is controller, 2 is autonmous, 3 is autonomous but picking up 
while True:
    if self.mode == 1:
        pass
    
    if  self.mode == 2:
        pass
    
