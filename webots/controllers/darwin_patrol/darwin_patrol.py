"""
DARwIn-OP controller for the "forest_fire" (wildfire) world.

Moves the robot using the Robotis DARwIn-OP walking gait (see walking.py) and
runs a simple autonomous patrol: walk forward, turn, repeat.  The movement
can be driven programmatically through the GaitController options (speed,
turn, strafe, stop).

Fire alarm: the DARwIn-OP carries a `Receiver` (named "receiver").  The fires
placed in the world emit a radio signal on channel 1 with a limited `range`;
when the robot is close enough to receive it, the controller speaks through
the `Speaker`, turns the LED eyes red and prints an alarm.

Usage:
  - Assigned as `controller "darwin_patrol"` in webots/worlds/forest_fire.wbt
  - Tested with Webots R2025a, Python controller, basicTimeStep 8.

If the Darwin-op Solid has `supervisor TRUE` this controller also prints its
world position every few seconds (useful for debugging/telemetry).
"""

from controller import Supervisor

from walking import Walking

#from views.view_people import view_people # línea nueva
#from views.view import View # línea nueva



# ============================================================================
# Gait controller wrapper
# ============================================================================

# Order of the 14 joints produced by Walking.process().
MOTOR_ORDER = [
    "PelvYR", "PelvR", "LegUpperR", "LegLowerR", "AnkleR", "FootR",     # right leg
    "PelvYL", "PelvL", "LegUpperL", "LegLowerL", "AnkleL", "FootL",     # left leg
    "ShoulderR", "ShoulderL",                                           # arms
]


class GaitController:
    """Drives the DARwIn-OP with the Robotis walking gait."""

    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step

        self.motors = {}
        for name in MOTOR_ORDER:
            motor = robot.getDevice(name)
            if motor is None:
                raise RuntimeError("Darwin-op motor not found: %s" % name)
            self.motors[name] = motor

        self.gyro = robot.getDevice("Gyro")
        if self.gyro is None:
            raise RuntimeError("Darwin-op Gyro not found")
        self.gyro.enable(time_step)

        self.walking = Walking()
        self.walking.initialize()

    # -- high level commands (same naming as RobotisOp2GaitManager) -------

    def set_speed(self, x=0.0, y=0.0, a=0.0):
        """Set walking amplitudes in [-1, 1].

        x: forward/back, y: lateral, a: rotation (turn).
        """
        self.walking.x_move_amplitude = max(-1.0, min(1.0, x)) * 20.0
        self.walking.y_move_amplitude = max(-1.0, min(1.0, y)) * 40.0
        self.walking.a_move_amplitude = max(-1.0, min(1.0, a)) * 50.0

    def start(self):
        self.robot.step(self.time_step)
        self.walking.start()

    def stop(self):
        self.walking.stop()

    def tick(self):
        """Advance the simulation one step and apply one gait tick.

        Returns False when the simulation terminated (robot instance gone).
        """
        if self.robot.step(self.time_step) == -1:
            return False

        gyro = self.gyro.getValues()
        self.walking.rl_gyro = gyro[0] - 512.0
        self.walking.fb_gyro = gyro[1] - 512.0

        self.walking.process()

        values = self.walking.joint_values
        for name, value in zip(MOTOR_ORDER, values):
            self.motors[name].setPosition(self.walking.value_to_position(value))
        return True

    def hold(self, seconds):
        """Run the gait at idle (zero amplitude) for a while, e.g. to stand up."""
        start = self.robot.getTime()
        while self.robot.getTime() - start < seconds:
            if not self.tick():
                return False
        return True


# ============================================================================
# Simple patrol schedule
# ============================================================================

class PatrolSchedule:
    """A cyclic sequence of (duration, x, y, a) motion commands."""

    def __init__(self, commands):
        self.commands = commands
        self.index = 0
        self.t0 = None

    def reset(self, now):
        self.index = 0
        self.t0 = now

    def update(self, now):
        duration, _, _, _ = self.commands[self.index]
        if self.t0 is None:
            self.t0 = now
        elapsed = now - self.t0
        if elapsed >= duration:
            self.index = (self.index + 1) % len(self.commands)
            self.t0 = now
            elapsed = 0.0
        _, x, y, a = self.commands[self.index]
        return x, y, a, elapsed


# ============================================================================
# Fire alarm
# ============================================================================

class Monitor:
    """Unifica la detección de fuego y personas en un solo Receiver."""

    ALARM_INTERVAL = 8.0   # mínimo de segundos entre alarmas de fuego
    CLEAR_DELAY = 12.0     # segundos sin señal de fuego antes de volver LEDs a verde

    def __init__(self, robot, time_step):
        self.receiver = robot.getDevice("receiver")
        if self.receiver is not None:
            self.receiver.enable(time_step)

        self.speaker = robot.getDevice("Speaker")
        self.eye = robot.getDevice("EyeLed")
        self.head = robot.getDevice("HeadLed")

        self.last_alarm = -self.ALARM_INTERVAL
        self.last_fire_signal = float("-inf")
        self.people_count = 0
        self.last_people_count = -1

    def check(self, now):
        if self.receiver is None:
            return

        fire_detected = False
        people_detected = 0

        while self.receiver.getQueueLength() > 0:
            message = self.receiver.getString()
            self.receiver.nextPacket()

            if message == "FIRE":
                fire_detected = True
            elif message == "HUMAN":
                people_detected += 1

        # --- Fuego ---
        if fire_detected:
            self.last_fire_signal = now
            if now - self.last_alarm >= self.ALARM_INTERVAL:
                self.last_alarm = now
                self._raise_fire_alarm()
        elif now - self.last_fire_signal >= self.CLEAR_DELAY:
            if self.eye is not None:
                self.eye.set(0x00FF00)
            if self.head is not None:
                self.head.set(0x00FF00)

        # --- Personas ---
        self.people_count = people_detected
        if self.people_count != self.last_people_count:
            print(f"[vision] personas vistas: {self.people_count}")
            self.last_people_count = self.people_count

    def _raise_fire_alarm(self):
        print("[alarm] fire detected near the robot")
        if self.speaker is not None:
            self.speaker.speak("Fire detected! Fire, fire!", 1.0)
        if self.eye is not None:
            self.eye.set(0xFF0000)
        if self.head is not None:
            self.head.set(0xFF0000)



# ============================================================================
# Main
# ============================================================================

def report_position(robot):
    """Print the robot world position (only works when supervisor is enabled)."""
    if not robot.supervisor:
        return
    try:
        pos = robot.getSelf().getPosition()
        print("[pos] x=%.2f y=%.2f z=%.2f" % (pos[0], pos[1], pos[2]))
    except Exception:
        pass


def main():
    print("Darwin-op patrol controller started (forest_fire world)")

    # A Supervisor instance behaves like a Robot (devices, step, time...), and
    # additionally supports getSelf()/position reporting when the Darwin-op has
    # its `supervisor` field set (e.g. for verification/telemetry runs).
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())

    #vision = View(robot, time_step) # línea nueva

    gait = GaitController(robot, time_step)

    robot.step(time_step)

    # ------------------------------------------------------------
    # 1) Settle the robot into the walking stance
    # ------------------------------------------------------------
    print("Assuming the walking stance...")
    if not gait.hold(2.0):
        return

    # ------------------------------------------------------------
    # 2) Continuous patrol
    # ------------------------------------------------------------
    print("Starting patrol")

    # (duration in s, x, y, a)
    #   x>0 walk forward, x<0 backwards, a<0 turn right, a>0 turn left
    schedule = PatrolSchedule([
        (25.0, 0.7, 0.0, 0.0),   # walk forward
        (8.0, 0.0, 0.0, 0.45),   # turn left
        (25.0, 0.7, 0.0, 0.0),   # walk forward
        (8.0, 0.0, 0.0, -0.45),  # turn right
    ])
    schedule.reset(robot.getTime())

    alarm = Monitor(robot, time_step)

    gait.start()

    last_report = robot.getTime()
    while True:

        #view_people(vision) # línea nueva

        now = robot.getTime()
        x, y, a, _ = schedule.update(now)
        gait.set_speed(x, y, a)

        if not gait.tick():
            return

        alarm.check(robot.getTime())

        if now - last_report >= 5.0:
            last_report = now
            report_position(robot)


if __name__ == "__main__":
    main()