from controller import Robot
import math

robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())


# ============================================================
# Get robot motors
# ============================================================

def get_motor(name):
    motor = robot.getDevice(name)

    if motor is None:
        print(f"[WARNING] Motor not found: {name}")

    return motor


motors = {}

motor_names = [
    # Head
    "HeadPan",
    "HeadTilt",

    # Left arm
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbow",

    # Right arm
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbow",

    # Left leg
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",

    # Right leg
    "RHipYawPitch",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
]

for name in motor_names:
    motors[name] = get_motor(name)


# ============================================================
# Utility functions
# ============================================================

def set_position(name, degrees):

    motor = motors.get(name)

    if motor is None:
        return

    motor.setPosition(math.radians(degrees))


def wait(seconds):

    steps = int(seconds * 1000 / TIME_STEP)

    for _ in range(steps):

        if robot.step(TIME_STEP) == -1:
            return False

    return True


# ============================================================
# Initial standing position
# ============================================================

def stand():

    print("Standing...")

    positions = {

        # Head
        "HeadPan": 0,
        "HeadTilt": 0,

        # Arms
        "LShoulderPitch": 0,
        "LShoulderRoll": 5,
        "LElbow": -30,

        "RShoulderPitch": 0,
        "RShoulderRoll": -5,
        "RElbow": 30,

        # Left leg
        "LHipYawPitch": 0,
        "LHipRoll": 0,
        "LHipPitch": 0,
        "LKneePitch": 0,
        "LAnklePitch": 0,
        "LAnkleRoll": 0,

        # Right leg
        "RHipYawPitch": 0,
        "RHipRoll": 0,
        "RHipPitch": 0,
        "RKneePitch": 0,
        "RAnklePitch": 0,
        "RAnkleRoll": 0,
    }

    for name, position in positions.items():
        set_position(name, position)

    wait(2.0)


# ============================================================
# Left step
# ============================================================

def step_left():

    print("Step LEFT")

    # Shift the body weight slightly to the left
    set_position("LHipRoll", -3)
    set_position("RHipRoll", -3)

    set_position("LAnkleRoll", 3)
    set_position("RAnkleRoll", 3)

    wait(0.3)

    # Slightly lift the right leg
    set_position("RHipPitch", -8)
    set_position("RKneePitch", 18)
    set_position("RAnklePitch", -10)

    wait(0.3)

    # Move the right leg forward
    set_position("RHipPitch", 10)
    set_position("RKneePitch", 15)
    set_position("RAnklePitch", -5)

    wait(0.3)

    # Place the foot back on the ground
    set_position("RKneePitch", 0)
    set_position("RAnklePitch", 0)

    wait(0.3)


# ============================================================
# Right step
# ============================================================

def step_right():

    print("Step RIGHT")

    # Shift the body weight slightly to the right
    set_position("LHipRoll", 3)
    set_position("RHipRoll", 3)

    set_position("LAnkleRoll", -3)
    set_position("RAnkleRoll", -3)

    wait(0.3)

    # Slightly lift the left leg
    set_position("LHipPitch", -8)
    set_position("LKneePitch", 18)
    set_position("LAnklePitch", -10)

    wait(0.3)

    # Move the left leg forward
    set_position("LHipPitch", 10)
    set_position("LKneePitch", 15)
    set_position("LAnklePitch", -5)

    wait(0.3)

    # Place the foot back on the ground
    set_position("LKneePitch", 0)
    set_position("LAnklePitch", 0)

    wait(0.3)


# ============================================================
# Arm movement
# ============================================================

def arms_swing(direction):

    amplitude = 10

    set_position(
        "LShoulderPitch",
        direction * amplitude
    )

    set_position(
        "RShoulderPitch",
        -direction * amplitude
    )


# ============================================================
# Main control loop
# ============================================================

stand()

phase = 1

print("Starting patrol movement...")

while robot.step(TIME_STEP) != -1:

    if phase == 1:

        arms_swing(1)

        step_left()

        phase = 2

    else:

        arms_swing(-1)

        step_right()

        phase = 1