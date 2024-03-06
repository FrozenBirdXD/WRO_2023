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
grabber_motor = Motor(Port.D)
height_motor = Motor(Port.C)
# Limits
left_motor.control.limits(1500, 10000, 100)
right_motor.control.limits(1500, 10000, 100)
grabber_motor.control.limits(1500, 10000, 100)
height_motor.control.limits(1500, 10000, 100)

# Sensors
left_color_sensor = ColorSensor(Port.S4)
right_color_sensor = ColorSensor(Port.S2)
main_color_sensor = ColorSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S1)


# ------------------------------------------------
# just initialize global objects

mnr = DriveController(left_motor, right_motor, DriveController.WHEEL_DIAMETER)
grabber = Grabber(
    Grabber.OPEN_GRABBER,
    Grabber.CLOSE_GRABBER,
    Grabber.OPEN_GRABBER_FULL,
    Grabber.HEIGHT_DOWN,
    Grabber.HEIGHT_UP_1,
    Grabber.HEIGHT_UP_2,
    Grabber.HEIGHT_UP_3,
    grabber_motor,
    height_motor,
)

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


def straighten():
    left_value = left_color_sensor.reflection()
    right_value = right_color_sensor.reflection()

    while 25 < right_value < 30 and 25 < left_value < 30:
        while right_value > 30 and left_value > 30:
            mnr.drive(50, 0)
            left_value = left_color_sensor.reflection()
            right_value = right_color_sensor.reflection()

        while right_value < 30 and left_value < 30:
            mnr.drive(-50, 0)
            left_value = left_color_sensor.reflection()
            right_value = right_color_sensor.reflection()

        while right_value > 30:
            mnr.drive(50, 100)
            right_value = right_color_sensor.reflection()

        while left_value > 30:
            mnr.drive(50, -100)
            left_value = left_color_sensor.reflection()

        left_value = left_color_sensor.reflection()
        right_value = right_color_sensor.reflection()

def gyro_turn(deg):
    gyro_sensor.reset_angle(0)
    if deg > 0:
        dire = -1
    elif deg < 0:
        dire = 1

    while gyro_sensor.angle() < abs(deg * 0.963):
        mnr.drive(300, 1000 * dire)

    mnr.stop()


if __name__ == "__main__":
    # main program logic
    # mnr.drive(150,400)
    # gyro_turn(90, "right")
    # gyro_turn(-90)
    while True:
        line_tracer("left")

