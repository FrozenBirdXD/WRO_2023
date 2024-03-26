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
from grabber import Grabber
from drive_controller import DriveController

import time
# -----------------------------------------------------------------------

# Initialize EV3
ev3 = EV3Brick()

# Motors
left_motor = Motor(Port.A, positive_direction = Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
graber_motor = Motor(Port.D)
height_motor = Motor(Port.C)
# Limits
left_motor.control.limits(1500, 5000, 100)
right_motor.control.limits(1500, 5000, 100)
graber_motor.control.limits(1500, 1500, 100)
height_motor.control.limits(1500, 1500, 100)

# Sensors
left_color_sensor = ColorSensor(Port.S4)
right_color_sensor = ColorSensor(Port.S2)
main_color_sensor = ColorSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S1)


# ------------------------------------------------
# just initialize global objects

mnr = DriveController(left_motor, right_motor, DriveController.WHEEL_DIAMETER, left_color_sensor, right_color_sensor, gyro_sensor)
graber = Grabber(height_motor, graber_motor)

# ----------------------------------------------------------------------------------------------

def check_sensors():
    while True:
        print("left")
        print(left_color_sensor.reflection())
        print("right")
        print(right_color_sensor.reflection())


def open_pipe():
    graber.graber_open_for_pipe_because_just_because()
    graber.height_complete_breakdown()
    graber.graber_open()
    mnr.drive(250,400)
    graber.yalla()
    wait(400)
    mnr.stop()


if __name__ == "__main__":
    #drive to first block
    mnr.drive_distance(500,190)
    mnr.turn(300, 90)
    mnr.drive_distance(500,250)
    mnr.turn(300, 90)
    mnr.straighten("front")
    mnr.drive_distance(500,100)

    #grab first block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    #drive to second block
    mnr.drive_distance(300, -70)
    mnr.turn(250, -90)
    mnr.drive_distance(300, 90)
    mnr.turn(250, 90)
    mnr.straighten("back")
    mnr.drive_distance(300, 75)

    #grab second block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    
    #grab first stack of blocks
    graber.graber_ready()
    graber.height_up()
    mnr.turn(200, 20)
    right_motor.run_angle(speed=200, rotation_angle=20, then=Stop.HOLD, wait=False)
    mnr.drive_distance(300, 100)

    #drive to thrid block
    graber.height_carry()
    mnr.drive_distance(300, -150)
    mnr.turn(200, -20)

    mnr.turn(200, -90)
    mnr.drive_distance(300, 80)
    mnr.turn(200, 90)

    mnr.straighten("back")
    mnr.drive_distance(300, 75)

    #grab third block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    #drive to fourth block
    mnr.drive_distance(300, -70)
    mnr.turn(200, -90)
    mnr.drive_distance(300, 90)
    mnr.turn(200, 90)
    mnr.straighten("back")
    mnr.drive_distance(300, 75)

    #grab fourth block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    #grab second stack of blocks
    graber.graber_ready()
    graber.height_up()
    mnr.turn(200, -20)
    right_motor.run_angle(speed=200, rotation_angle=-20, then=Stop.HOLD, wait=False)
    mnr.drive_distance(300, 100)

    graber.height_carry()
    mnr.drive_distance(300, -150)
    mnr.turn(200, -20)
