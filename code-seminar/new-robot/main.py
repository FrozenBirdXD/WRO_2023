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

# Constants
WHEEL_DIAMETER = 62.4  # in millimeters
Kp = 0.1  # can be larger
Ki = 0.0000  # small
Kd = 0.0  # small
TARGET_REFLECTION = 27

# Initialize EV3
ev3 = EV3Brick()

# Motors
left_motor = Motor(Port.A)
right_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
graber = Motor(Port.D)
height = Motor(Port.C)
# Limits
left_motor.control.limits(1500, 1000000000, 8000)
right_motor.control.limits(1500, 1000000000, 8000)
graber.control.limits(1500, 1000000000, 8000)
height.control.limits(1500, 1000000000, 8000)

# Sensors
left_color_sensor = ColorSensor(Port.S4)
right_color_sensor = ColorSensor(Port.S2)
main_color_sensor = ColorSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S1)


# Drivebase
class DriveController:
    def __init__(self, left_motor, right_motor, wheel_diameter):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_diameter = wheel_diameter

    def drive(self, speed, turn_rate):
        left_speed = speed
        right_speed = speed

        if turn_rate > 0:
            right_speed *= (1000 - (turn_rate * 2)) / 1000
        elif turn_rate < 0:
            left_speed *= (1000 + (turn_rate * 2)) / 1000

        self.left_motor.run(left_speed)
        self.right_motor.run(-right_speed)

    def stop(self):
        self.left_motor.hold()
        self.right_motor.hold()

    def straight(self, speed, distance):
        angle = 360 * (distance / ((self.wheel_diameter / 2) * 2 * 3.1415926))
        self.left_motor.run_angle(speed, angle, then=Stop.HOLD, wait=True)
        self.right_motor.run_angle(-speed, angle, then=Stop.HOLD, wait=True)


mnr = DriveController(left_motor, right_motor, WHEEL_DIAMETER)

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


# values for graber - constants
GRAD_OPEN_FULL = 180
GRAB_OPEN = 0
GRAB_CLOSE = 90

HEIGHT_UP_1 = 40
HEIGHT_UP_2 = 80
HEIGHT_UP_3 = 180
HEIGHT_DOWN = 0


class Grabber:
    def __init__(
        self,
        grab_open,
        grab_close,
        grab_open_full,
        height_down,
        height_up_1,
        height_up_2,
        height_up_3,
        graber_motor,
        height_motor,
    ):
        self.grab_open = grab_open
        self.grab_close = grab_close
        self.grab_open_full = grab_open_full
        self.height_down = height_down
        self.height_up_1 = height_up_1
        self.height_up_2 = height_up_2
        self.height_up_3 = height_up_3
        self.graber_motor = graber_motor
        self.height_motor = height_motor

    def pickup(self, level):
        if level == 1:
            self.graber_motor.run_target(100, self.grab_open, then=Stop.HOLD, wait=True)
            self.height_motor.run_target(
                100, self.height_down, then=Stop.HOLD, wait=True
            )
            self.graber_motor.run_target(
                100, self.grab_close, then=Stop.HOLD, wait=True
            )
            self.height_motor.run_target(
                100, self.height_up_2, then=Stop.HOLD, wait=True
            )
        elif level == 2:
            self.height_motor.run_target(
                100, self.height_up_1, then=Stop.HOLD, wait=True
            )
            self.graber_motor.run_target(100, self.grab_open, then=Stop.HOLD, wait=True)
            self.height_motor.run_target(
                100, self.height_down, then=Stop.HOLD, wait=True
            )
            self.graber_motor.run_target(
                100, self.grab_close, then=Stop.HOLD, wait=True
            )
            self.height_motor.run_target(
                100, self.height_up_2, then=Stop.HOLD, wait=True
            )

    def drop(self, level):
        if level == 1:
            self.height_motor.run_target(
                100, self.height_down, then=Stop.HOLD, wait=True
            )
            self.graber_motor.run_target(100, self.grab_open, then=Stop.HOLD, wait=True)
            self.height_motor.run_target(
                100, self.height_up_2, then=Stop.HOLD, wait=True
            )
        elif level == 2:
            self.height_motor.run_target(
                100, self.height_up_1, then=Stop.HOLD, wait=True
            )
            self.graber_motor.run_target(100, self.grab_open, then=Stop.HOLD, wait=True)
            self.height_motor.run_target(
                100, self.height_up_2, then=Stop.HOLD, wait=True
            )

    def pipe(self):
        self.height_motor.run_target(100, self.height_down, then=Stop.HOLD, wait=True)
        self.graber_motor.run_target(
            100, self.grab_open_full, then=Stop.HOLD, wait=True
        )
        self.height_motor.run_target(100, self.height_up_2, then=Stop.HOLD, wait=True)

    def open(self):
        self.height_motor.run_target(100, self.height_up_3, then=Stop.HOLD, wait=True)

    def close(self):
        self.height_motor.run_target(100, self.height_up_1, then=Stop.HOLD, wait=True)
        
grabber = Grabber(
        GRAB_OPEN,
        GRAB_CLOSE,
        GRAD_OPEN_FULL,
        HEIGHT_DOWN,
        HEIGHT_UP_1,
        HEIGHT_UP_2,
        HEIGHT_UP_3,
        graber,
        height,
    )


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
    pass
