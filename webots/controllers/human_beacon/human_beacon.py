"""
Human beacon: emite un pulso de radio en canal 1 para que el DARwIn-OP
(lo cual lleva un Receiver en canal 1) detecte la presencia de personas
cuando se acerca. El campo `range` del Emitter define la distancia de detección.
"""

from controller import Robot

robot = Robot()
time_step = int(robot.getBasicTimeStep())
emitter = robot.getDevice("people_emitter")

step_count = 0
while robot.step(time_step) != -1:
    step_count += 1
    if emitter is not None:
        emitter.send(b"HUMAN")
