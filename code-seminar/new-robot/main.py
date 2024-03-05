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
Kp = 0.1  # can be larger
Ki = 0.0000  # small
Kd = 0.0  # small
TARGET_REFLECTION = 27

# Initialize EV3
ev3 = EV3Brick()

# Motors
left_motor = Motor(Port.A)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
grabber_motor = Motor(Port.D)
height_motor = Motor(Port.C)
# Limits
left_motor.control.limits(1500, 1000000000, 8000)
right_motor.control.limits(1500, 1000000000, 8000)
grabber_motor.control.limits(1500, 1000000000, 8000)
height_motor.control.limits(1500, 1000000000, 8000)

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
    global last_error

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
    mnr.drive(200, correction * multiplier)

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


# If the angle reading don't precisely match the target angle
# (quite common), the while loop may become infinite, as it depends on an exact match
# -> error we saw yesteray
# -Matthew
def gyro_turn(deg):
    gyro_sensor.reset_angle(0)
    if deg > 0:
        dire = 1
    elif deg < 0:
        dire = -1

    while gyro_sensor.angle() != deg:
        mnr.drive(100, 1000 * dire)

    mnr.stop()


# Random zeug von Matthew
def gyro_turn(degrees, direction):
    gyro_sensor.reset_angle(0)

    # Determine direction multiplier
    if direction == "right":
        multiplier = -1
    elif direction == "left":
        multiplier = 1
    else:
        raise ValueError("Invalid direction")

    target_angle = degrees * multiplier
    speed = 50

    # Turning
    while abs(gyro_sensor.angle()) < abs(target_angle):
        # Adjust motor speeds based on remaining angle to turn
        correction = (abs(target_angle) - abs(gyro_sensor.angle())) * 2
        left_speed = speed - correction
        right_speed = speed + correction

        # --------------------
        # without speed adjustment
        # left_motor.run(50 * multiplier)
        # right_motor.run(-50 * multiplier)
        # --------------------

        left_motor.run(left_speed)
        right_motor.run(-right_speed)

    left_motor.hold()
    right_motor.hold()


if __name__ == "__main__":
    # main program logic
    mnr.drive(100, 0)
    
    pass
