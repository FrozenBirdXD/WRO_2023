#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    InfraredSensor,
    UltrasonicSensor,
    GyroSensor,
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# custom files
from graber import Graber
from drive_controller import DriveController

import time

# -----------------------------------------------------------------------

# Initialize EV3
ev3 = EV3Brick()

# Motors
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
graber_motor = Motor(Port.D)
height_motor = Motor(Port.C)
# Limits
left_motor.control.limits(1500, 2200, 100)
right_motor.control.limits(1500, 2200, 100)
graber_motor.control.limits(1500, 4000, 100)
height_motor.control.limits(1500, 1500, 100)

# Sensors
left_color_sensor = ColorSensor(Port.S4)
right_color_sensor = ColorSensor(Port.S2)
main_color_sensor = ColorSensor(Port.S3)

# ------------------------------------------------
# just initialize global objects

mnr = DriveController(
    left_motor,
    right_motor,
    DriveController.WHEEL_DIAMETER,
    left_color_sensor,
    right_color_sensor,
)
graber = Graber(height_motor, graber_motor)

# ----------------------------------------------------------------------------------------------

def open_pipe():
    graber.graber_open_for_pipe_because_just_because()
    graber.height_complete_breakdown()
    graber.graber_open()
    mnr.drive(250, 400)
    graber.yalla()
    wait(400)
    mnr.stop()


def start():
    mnr.drive_distance(500, 40)
    mnr.u_turn(600, 235, 0.5)


def get_red_yellow():
    # grab first block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    # shift to next block
    mnr.shift(-100, -130, 1200)
    mnr.drive_distance(1000, 130)

    # grab second block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    # grab first stack of blocks
    graber.graber_ready()
    graber.height_up_aggressive()
    mnr.turn(200, 20)
    right_motor.run_angle(speed=200, rotation_angle=20, then=Stop.HOLD, wait=False)
    mnr.drive_distance(300, 110)
    graber.height_carry()
    mnr.drive_distance(700, -110)
    mnr.turn(200, -20)

    mnr.shift(-95, -130, 1200)
    mnr.drive_distance(1000, 130)

    # grab third block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    # drive to fourth block
    mnr.shift(-95, -130, 1200)
    mnr.drive_distance(1000, 130)

    # grab fourth block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    # put blocks on color
    graber.height_carry()


def place_red_yellow():
    # straighten with border
    mnr.turn(500, 180)
    mnr.drive_distance(700, -175)

    # put back red
    mnr.turn_after(210, 435, "left")
    graber.height_up()
    mnr.drive_distance(1000, -60)

    # put back yellow
    mnr.shift(-1, -265, 1200)
    graber.height_1()
    graber.graber_open()
    graber.height_up_aggressive()


def rth():
    # drive to line
    mnr.shift(290, -345, 1200)
    mnr.line_tracer_distance("right", 215)
    wait(200)
    # drive to first block - red
    mnr.turn(300, 91)
    mnr.drive_distance(300, 30)


def stack_yellow():
    # stack yellow blocks
    mnr.drive_distance(250, 60)
    mnr.turn(300, 180)

    # drive to stack
    mnr.line_tracer_distance("left", 190)
    mnr.drive_distance(300, 20)
    graber.height_4()
    graber.graber_ready()
    graber.height_carry()

    mnr.drive_distance(250, -110)


def stack_red():
    # put down red blocks
    mnr.turn(300, 90)
    graber.height_up()
    mnr.drive_distance(1000, -90)
    mnr.shift(-50, -70, 1200)
    mnr.drive_distance(1000, 50)
    # pick up blocks with graber
    graber.height_1()
    graber.graber_close()
    graber.height_carry()
    # drive until stack
    mnr.drive_distance(1000, 300)
    mnr.turn(300, -90)
    mnr.drive_distance(300, 110)
    mnr.drive_distance(300, -55)
    # put blocks on stack
    graber.height_4()
    graber.graber_ready()
    graber.height_carry()

    mnr.drive_distance(250, -100)


if __name__ == "__main__":
    start()
    get_red_yellow()
    place_red_yellow()
    rth()
    get_red_yellow()
    stack_yellow()
    stack_red()
