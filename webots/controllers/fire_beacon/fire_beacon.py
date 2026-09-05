"""Fire beacon: emits a radio pulse on channel 1 so that the patrol robot
(which carries a Receiver on channel 1) detects the fire when it gets close.
The Emitter `range` field defines the detection distance."""

from controller import Robot

robot = Robot()
time_step = int(robot.getBasicTimeStep())
emitter = robot.getDevice("emitter")

step_count = 0
while robot.step(time_step) != -1:
    step_count += 1
    if emitter is not None and step_count % 8 == 0:
        emitter.send(b"FIRE")