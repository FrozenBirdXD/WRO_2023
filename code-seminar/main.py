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
    mnr.u_turn(600, 235, 0.495)

def start_2():
    mnr.drive_distance(400,80)
    mnr.turn(300,90)
    mnr.drive_distance(300,50)
    mnr.line_tracer_distance("right", 205)
    mnr.turn(300, 91)
    mnr.drive_distance(300, 30)


def get_red_yellow():
    # grab first block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    # shift to next block
    mnr.shift(-95, -130, 1000)
    mnr.drive_distance(1000, 130)

    # grab second block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    # grab first stack of blocks
    graber.graber_ready()
    graber.height_up_aggressive()
    mnr.turn(200, 22)
    right_motor.run_angle(speed=200, rotation_angle=20, then=Stop.HOLD, wait=False)
    mnr.drive_distance(300, 120)
    graber.height_carry()
    mnr.drive_distance(700, -120)
    mnr.turn(200, -22)

    mnr.shift(-92, -130, 1000)
    mnr.drive_distance(1000, 130)

    # grab third block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    # drive to fourth block
    mnr.shift(-92, -130, 1000)
    mnr.drive_distance(1000, 130)

    # grab fourth block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    # put blocks on color
    graber.height_carry()


def get_red_yellow_2():
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
    # straighten with border
    # ------------------ old 
    # mnr.turn(500, 180)
    # mnr.drive_distance(500, 425)
    # mnr.turn(500, -86)
    # mnr.drive_distance(500, 340)
    # -------------------
    # ---------------- old old
    # mnr.drive_distance(700, -160)

    # put back red
    # mnr.turn_after(220, 475, "left")
    # -----------------

    mnr.u_turn(500, 200, 0.5)
    mnr.line_tracer_distance("left", 150, 250)
    mnr.drive_distance(260, 275)
    mnr.turn(400, -90)
    mnr.drive_distance(500, 125)

    graber.height_up()
    mnr.drive_distance(1000, -60)
    
    # put back yellow
    mnr.shift(-1, -270, 1000)
    graber.height_1()
    graber.graber_open()
    graber.height_up_aggressive()


def rth():
    # drive to line
    mnr.shift(275, -335, 1000)
    mnr.line_tracer_distance("right", 215)
    wait(200)
    # drive to first block - red
    mnr.turn(300, 91)
    mnr.drive_distance(300, 15)


def stack_yellow():
    # stack yellow blocks
    mnr.drive_distance(250, 60)
    mnr.turn(300, 180)

    # drive to stack
    mnr.line_tracer_distance("right", 190)
    mnr.drive_distance(300, 50)
    mnr.drive_distance(300, -60)
    graber.height_4()
    graber.graber_ready()
    graber.height_carry()

    mnr.drive_distance(250, -110)


def stack_red():
    # put down red blocks
    mnr.turn(300, 90)
    graber.height_up()
    mnr.drive_distance(1000, -90)
    mnr.shift(-57, -70, 1000)
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
    #collects trash 1 and 2

    mnr.drive_distance(1200, -200)
    mnr.shift(210, 300, 800)
    mnr.u_turn(1200,-480,0.25)

    mnr.drive_distance(1500,680)
    mnr.u_turn(1200,-240,0.25)
    mnr.drive_distance(1200, 100)
    mnr.u_turn(1200,240,0.25)

    mnr.turn(800, 5)
    mnr.drive_distance(1200, 150)
    mnr.u_turn(1000,-200,0.47)
    # brings away trash
    mnr.u_turn(1200,-240,0.25)
    mnr.drive_distance(1200,450)
    mnr.u_turn(1200,240,0.242)  

    mnr.drive_distance(1200, 1150)
    mnr.u_turn(1200, -260, 0.25)
    mnr.drive_distance(1000,400)

def trash_2():
    mnr.drive_distance(1200, -200)
    mnr.shift(210, 300, 800)
    mnr.drive_distance(1000,350)
    graber.height_1()
    graber.graber_close()
    graber.height_up()
    mnr.u_turn(-600,100,0.25)
    mnr.drive_distance(1000,200)
    mnr.line_tracer_distance("right",300)
    mnr.u_turn(600,-350,0.25)
    mnr.turn(300,90)
    mnr.drive_distance(1000,500)
    graber.height_carry()
    graber.graber_open()
    graber.height_up()
    mnr.drive_distance(300,-100)
    mnr.u_turn(600,-150,0.25)
    mnr.u_turn(600,150,0.25)
    mnr.drive_distance(1500,1000)


if __name__ == "__main__":
    start_2()
    get_red_yellow()
    place_red_yellow()
    rth()
    get_red_yellow()
    stack_yellow()
    stack_red()
    trash()
    # trash_2()


