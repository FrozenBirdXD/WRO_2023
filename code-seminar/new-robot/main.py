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

# Constants
Kp = 10  # can be larger
Ki = 0.0000  # small
Kd = 0.0  # small
TARGET_REFLECTION = 27

# Initialize EV3
ev3 = EV3Brick()

# Motors
left_motor = Motor(Port.A, positive_direction = Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction= Direction.COUNTERCLOCKWISE)
graber_motor = Motor(Port.D)
height_motor = Motor(Port.C)
# Limits
left_motor.control.limits(1500, 10000, 100)
right_motor.control.limits(1500, 10000, 100)
graber_motor.control.limits(1500, 10000, 100)
height_motor.control.limits(1500, 1500, 100)

# Sensors
left_color_sensor = ColorSensor(Port.S4)
right_color_sensor = ColorSensor(Port.S2)
main_color_sensor = ColorSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S1)


# ------------------------------------------------
# just initialize global objects

mnr = DriveController(left_motor, right_motor, DriveController.WHEEL_DIAMETER)
graber = Grabber(height_motor, graber_motor)

# ----------------------------------------------------------------------------------------------


def line_tracer(side: str):
    last_error = 0

    target = TARGET_REFLECTION
    integral = 0

    if side == "right":
        value = right_color_sensor.reflection()
        multiplier = 1
    elif side == "left":
        value = left_color_sensor.reflection()
        multiplier = -1
    else:
        raise ValueError("Invalid side")

    error = target - value
    integral += error
    derative = error - last_error

    correction = (error * Kp) + (integral * Ki) + (derative * Kd)
    mnr.drive(500, correction * multiplier)

    last_error = error

def check_sensors():
    while True:
        print("left")
        print(left_color_sensor.reflection())
        print("right")
        print(right_color_sensor.reflection())

def straighten():
    K = 7
    left = left_color_sensor.reflection()
    right = right_color_sensor.reflection()
    while True:
        left = left_color_sensor.reflection() - 3
        error = TARGET_REFLECTION - left
        speed_left = (error * K)
        right = right_color_sensor.reflection()
        error = TARGET_REFLECTION - right
        speed_right = (error * K)
        if left == right == TARGET_REFLECTION:
            mnr.stop()
            break
        left_motor.run(-speed_left)
        right_motor.run(speed_right)

def open_pipe():
    graber.graber_open_for_pipe_because_just_because()
    graber.height_complete_breakdown()
    graber.graber_open()
    mnr.drive(250,400)
    graber.yalla()
    wait(400)
    mnr.stop()

def gyro_turn(deg):
    gyro_sensor.reset_angle(0)
    if deg > 0:
        dire = -1
    elif deg < 0:
        dire = 1

    while abs(gyro_sensor.angle()) < abs(deg * 0.964):
        mnr.drive(300, 1000 * dire)

    mnr.stop()


if __name__ == "__main__":

    # main program logic
    # mnr.drive(150,400)
    # gyro_turn(180)
    # while True:
    #     line_tracer("left")

    # height_motor.reset_angle(0)
    # height_motor.run_target(500, -500, then=Stop.HOLD, wait=True)
    # mnr.drive(400, 0)

    # graber.nach_hinten_digga()

    # graber.graber_close()
    # wait(1000)
    # graber.height_4()
    # wait(1000)
    # graber.height_down()
    # wait(1000)
    # graber.graber_close()
    # wait(1000)
    # graber.height_up()
    # graber.height_4()
    # wait(1000)
    # graber.height_down()
    # wait(1000)
    # graber.graber_close()
    # wait(1000)
    # graber.height_up()
    # straighten()
    # graber.graber_open_full()
    # mnr.drive_distance(400,96)
    # wait(1000)
    # gyro_turn(-90)
    # mnr.drive_distance(400,65)
    # wait(1000)
    # StopWatch.time()
    # while StopWatch.time() <= 300:
    #     line_tracer()
    # t_end = time.time() + 3
    # while time.time() < t_end:
    #     line_tracer("left")


    # straighten()
    # check_sensors()
    # mnr.drive(499, 0)
    open_pipe()
    pass

