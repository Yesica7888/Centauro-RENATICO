from controller import Robot
import math

robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())


# ============================================================
# Get motors
# ============================================================

motor_names = [
    "ShoulderR",
    "ShoulderL",
    "ArmUpperR",
    "ArmUpperL",
    "ArmLowerR",
    "ArmLowerL",
    "PelvYR",
    "PelvYL",
    "PelvR",
    "PelvL",
    "LegUpperR",
    "LegUpperL",
    "LegLowerR",
    "LegLowerL",
    "AnkleR",
    "AnkleL",
    "FootR",
    "FootL",
    "Neck",
    "Head",
]

motors = {
    name: robot.getDevice(name)
    for name in motor_names
}


# ============================================================
# Get position sensors
# ============================================================

knee_right_sensor = robot.getDevice("LegLowerRS")
knee_left_sensor = robot.getDevice("LegLowerLS")

knee_right_sensor.enable(TIME_STEP)
knee_left_sensor.enable(TIME_STEP)


# ============================================================
# Set motor position
# ============================================================

def set_motor(name, degrees):

    motors[name].setPosition(
        math.radians(degrees)
    )


# ============================================================
# Wait
# ============================================================

def wait(seconds):

    steps = int(seconds * 1000 / TIME_STEP)

    for _ in range(steps):

        if robot.step(TIME_STEP) == -1:
            return False

    return True


# ============================================================
# Move to standing position
# ============================================================

def stand_up():

    print("Standing up...")

    standing_pose = {

        # Pelvis
        "PelvR": 0,
        "PelvL": 0,

        "PelvYR": 0,
        "PelvYL": 0,

        # Legs
        "LegUpperR": 0,
        "LegUpperL": 0,

        "LegLowerR": 0,
        "LegLowerL": 0,

        # Ankles
        "AnkleR": 0,
        "AnkleL": 0,

        "FootR": 0,
        "FootL": 0,

        # Arms
        "ShoulderR": 0,
        "ShoulderL": 0,

        "ArmUpperR": 0,
        "ArmUpperL": 0,

        "ArmLowerR": 0,
        "ArmLowerL": 0,

        # Head
        "Neck": 0,
        "Head": 0,
    }

    for name, angle in standing_pose.items():

        set_motor(name, angle)

    # Give the motors time to reach the standing position

    wait(2.0)

    return True


# ============================================================
# Move the right leg forward
# ============================================================

def right_step():

    print("Right step")

    # --------------------------------------------------------
    # Shift the body weight to the left leg
    # --------------------------------------------------------

    set_motor("PelvR", -3)
    set_motor("PelvL", -3)

    set_motor("AnkleR", 3)
    set_motor("AnkleL", 3)

    wait(0.2)


    # --------------------------------------------------------
    # Bend the right knee
    # --------------------------------------------------------

    set_motor("LegLowerR", 12)

    wait(0.2)


    # --------------------------------------------------------
    # Move the right leg forward
    # --------------------------------------------------------

    set_motor("LegUpperR", 8)

    set_motor("AnkleR", -5)

    # Move the left arm backward
    set_motor("ShoulderL", -8)

    # Move the right arm forward
    set_motor("ShoulderR", 8)

    wait(0.3)


    # --------------------------------------------------------
    # Put the right foot on the ground
    # --------------------------------------------------------

    set_motor("LegLowerR", 0)

    set_motor("LegUpperR", 0)

    set_motor("AnkleR", 0)

    wait(0.2)


# ============================================================
# Move the left leg forward
# ============================================================

def left_step():

    print("Left step")

    # --------------------------------------------------------
    # Shift the body weight to the right leg
    # --------------------------------------------------------

    set_motor("PelvR", 3)
    set_motor("PelvL", 3)

    set_motor("AnkleR", -3)
    set_motor("AnkleL", -3)

    wait(0.2)


    # --------------------------------------------------------
    # Bend the left knee
    # --------------------------------------------------------

    set_motor("LegLowerL", 12)

    wait(0.2)


    # --------------------------------------------------------
    # Move the left leg forward
    # --------------------------------------------------------

    set_motor("LegUpperL", 8)

    set_motor("AnkleL", -5)

    # Move the left arm forward
    set_motor("ShoulderL", 8)

    # Move the right arm backward
    set_motor("ShoulderR", -8)

    wait(0.3)


    # --------------------------------------------------------
    # Put the left foot on the ground
    # --------------------------------------------------------

    set_motor("LegLowerL", 0)

    set_motor("LegUpperL", 0)

    set_motor("AnkleL", 0)

    wait(0.2)


# ============================================================
# Main
# ============================================================

print("Darwin-OP controller started")


# Initial simulation step

if robot.step(TIME_STEP) == -1:
    exit()


# Move the robot to the initial standing position

stand_up()

print("Robot should now be standing")

print("Starting walking loop...")


# ============================================================
# Walking loop
# ============================================================

while robot.step(TIME_STEP) != -1:

    # Move right leg forward

    right_step()

    # Move left leg forward

    left_step()