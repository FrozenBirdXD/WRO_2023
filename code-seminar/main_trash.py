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
back_color_sensor = ColorSensor(Port.S1)

# ------------------------------------------------
# just initialize global objects

mnr = DriveController(
    left_motor,
    right_motor,
    DriveController.WHEEL_DIAMETER,
    left_color_sensor,
    right_color_sensor,
    back_color_sensor,
)
graber = Graber(height_motor, graber_motor)

# ----------------------------------------------------------------------------------------------


def start():
    mnr.drive_distance(500, 40)
    mnr.u_turn(600, 240, 0.5)


def get_red_yellow():
    # grab first block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    # shift to next block
    mnr.shift(-97, -130, 1200)
    mnr.drive_distance(1000, 130)

    # grab second block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    # grab first stack of blocks
    graber.graber_ready()
    graber.height_up_aggressive()
    mnr.turn(200, 23)
    right_motor.run_angle(speed=200, rotation_angle=20, then=Stop.HOLD, wait=False)
    mnr.drive_distance(300, 120)
    graber.height_carry()
    mnr.drive_distance(700, -120)
    mnr.turn(200, -23)

    mnr.shift(-100, -130, 1200)
    mnr.drive_distance(1000, 130)

    # grab third block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    # drive to fourth block
    mnr.shift(-105, -130, 1200)
    mnr.drive_distance(1000, 130)

    # grab fourth block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    # put blocks on color
    graber.height_carry()


def place_red_yellow():

    mnr.turn(800, 175)
    # mnr.line_tracer_distance("right", 100)
    mnr.drive_distance(500,120)
    mnr.u_turn(800, 210, 0.25)
    graber.height_up()

    # put back yellow
    # mnr.shift(-1, -250, 1200)
    mnr.drive_distance(500,-290)
    mnr.turn(300,-20)
    graber.height_1()
    graber.graber_open()
    graber.height_up_aggressive()





def rth():
    mnr.u_turn(-500, -240, 0.485)
    # mnr.drive_distance(-900, 520)
    mnr.line_tracer_distance("back",310,-500)
    mnr.drive_distance(400,-200)
    mnr.line_tracer_distance("right", 190)
    wait(200)
    # drive to first block - red
    mnr.turn(300, 91)
    mnr.drive_distance(300, 15)


def stack_yellow():
    # stack yellow blocks
    # drive to stack
    mnr.u_turn(-800, -200, 0.25)
    mnr.line_tracer_distance("right", 350)
    mnr.turn(800, 90)
    mnr.line_tracer_distance("left", 190)

    mnr.drive_distance(300, 50)
    mnr.drive_distance(300, -60)
    graber.height_4()
    graber.graber_ready()
    graber.height_carry()

    mnr.drive_distance(250, -110)


def stack_red():
    # put down red blocks
    mnr.turn(300, -90)
    graber.height_up()
    mnr.drive_distance(1000, -90)
    mnr.shift(-60, -70, 1200)
    mnr.drive_distance(1000, 50)
    # pick up blocks with graber
    graber.height_1()
    graber.graber_close()
    graber.height_carry()
    # drive until stack
    mnr.drive_distance(1000, 310)
    mnr.turn(300, -90)
    mnr.drive_distance(300, 110)
    mnr.drive_distance(300, -55)
    # put blocks on stack
    graber.height_4()
    graber.graber_ready()
    graber.height_up()

def trash():
    mnr.turn(300,-155)
    mnr.drive_distance(1000,470)
    mnr.u_turn(600,-550,0.25)
    mnr.turn(300,-90)
    mnr.drive_distance(1500,1220)
    mnr.u_turn(600,-300,0.25)
    mnr.drive_distance(1000,330)
    mnr.turn(300,60)
    mnr.drive_distance(1000,380)
    mnr.u_turn(300,-250,0.18)

    #pipe
    mnr.u_turn(-300,-185,0.25)
    mnr.drive_distance(300,-100)
    graber.height_complete_breakdown()
    graber.graber_open_for_pipe_because_just_because()
    mnr.drive_distance(300,120)
    graber.height_4()
    mnr.turn(300, 30)
    
    













    #collects trash 1 and 2

    # mnr.drive_distance(1200, -200)
    # mnr.shift(210, 300, 800)
    # mnr.u_turn(1200,-480,0.25)

    # mnr.drive_distance(1500,700)
    # mnr.u_turn(1200,-240,0.25)
    # mnr.drive_distance(1200, 100)
    # mnr.u_turn(1200,240,0.25)

    # mnr.turn(800, 5)
    # mnr.drive_distance(1200, 150)
    # mnr.u_turn(1000,-200,0.47)
    # brings away trash
    # mnr.u_turn(1200,-240,0.25)
    # mnr.drive_distance(1200, 500)
    # mnr.u_turn(1200,240,0.25)  

    # mnr.drive_distance(1200, 1300)
    # mnr.u_turn(1200, -260, 0.25)


    # drive to pipe
    # mnr.drive_distance(1200,-50)
    # mnr.u_turn(1200, 5, 0.25)

    # open pipe
    # graber.graber_open_for_pipe_because_just_because()
    # graber.height_complete_breakdown()
    # mnr.drive_distance(1500, 300)

    # graber.height_4()
    # mnr.turn(300, 30)
    # mnr.drive_distance(1200, -200)
    # graber.height_up()


if __name__ == "__main__":
    start()
    get_red_yellow()
    place_red_yellow()
    # rth()
    # get_red_yellow()
    # stack_yellow()
    # stack_red()
    trash()




