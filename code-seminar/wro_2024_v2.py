#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

#Motors
left_motor = Motor(Port.A)
right_motor = Motor(Port.B, positive_direction = Direction.CLOCKWISE)
graber = Motor(Port.D)
height = Motor(Port.C)
left_motor.control.limits(1500,1000000000,8000)
right_motor.control.limits(1500,1000000000,8000)
graber.control.limits(1500,1000000000,8000)
height.control.limits(1500,1000000000,8000)

#Drivebase
class DRIVE:

    def __init__(self,left_motor,right_motor,wheel_diameter):
        self.left = left_motor
        self.right = right_motor
        self.diameter = wheel_diameter

    def drive(self,speed,turn_rate):
        if turn_rate > 0:
            left = speed 
            right = speed * ((1000-(turn_rate*2))/1000)
        elif trun_rate < 0:
            left = speed * ((1000+(turn_rate*2))/1000)
            right = speed
        else:
            left = speed
            right = speed

        self.left.run(left)
        self.right.run(-right)
    
    def stop(self):
        self.left.hold()
        self.right.hold()

    def straight(self,speed,distance):
        angle = 360 * (distance/((self.diameter/2)*2*π))
        self.left.run_angle(speed, angle, then=Stop.HOLD, wait=True)
        self.right.run_angle(-speed, angle, then=Stop.HOLD, wait=True)



mnr = DRIVE(left_motor,right_motor,wheel_diameter = 62.4) 
mnr2 = DriveBase(left_motor,right_motor,axle_track = 175,wheel_diameter = 62.4)

#Sensors
b_l_color = ColorSensor(Port.S4)
b_r_color = ColorSensor(Port.S2)
main_color = ColorSensor(Port.S3)
gyro = GyroSensor(Port.S1)


#testing








# Write your program here.

def line_tracer(side:str):
    target = 27
    integral = 0
    error = 0
    last_error = 0
    derative = 0

    Kp = 0.1 # can be larger
    Ki = 0.0000 # needs to be quite small
    Kd = 0.0# needs to be small too

    if side == "right":
        value = b_r_color.reflection()
        dire = 1
    elif side == "left":
        value = b_l_color.reflection()
        dire = -1

    error = target - value
    integral += error
    derative = error - last_error

    correction = (error * Kp)+(integral * Ki)+(derative * Kd)
    print(correction)
    print(value)
    mnr.drive(200,correction * dire)
    
    last_error = error



def straighten():
    left_value = b_l_color.reflection()
    right_value = b_r_color.reflection()

    while right_value > 25 and right_value < 30 and left_value > 25 and left_value < 30:

        while right_value > 30 and left_value > 30:
            mnr.drive(50,0)
            left_value = b_l_color.reflection()
            right_value = b_r_color.reflection()

        while right_value < 30 and left_value < 30:
            mnr.drive(-50,0)
            left_value = b_l_color.reflection()
            right_value = b_r_color.reflection()

        while right_value > 30:
            mnr.drive(50,100)
            right_value = b_r_color.reflection()

        while left_value > 30:
            mnr.drive(50,-100)
            left_value = b_l_color.reflection()
        
        left_value = b_l_color.reflection()
        right_value = b_r_color.reflection()

#values for graber
open_grb_full = 180
open_grb = 0
close_grb = 90

up_1 = 40
up_2 = 80
up_3 = 180
down = 0

def picup(level):
    if level == 1:
        graber.run_target(100, open_grb, then=Stop.HOLD, wait=True)
        height.run_target(100, down, then=Stop.HOLD, wait=True)
        graber.run_target(100, close_grb, then=Stop.HOLD, wait=True)
        height.run_target(100, up_2, then=Stop.HOLD, wait=True)
    if level == 2:
        height.run_target(100, up_1, then=Stop.HOLD, wait=True)
        graber.run_target(100, open_grb, then=Stop.HOLD, wait=True)
        height.run_target(100, down, then=Stop.HOLD, wait=True)
        graber.run_target(100, close_grb, then=Stop.HOLD, wait=True)
        height.run_target(100, up_2, then=Stop.HOLD, wait=True)

def drop(level):
    if level == 1:
        height.run_target(100, down, then=Stop.HOLD, wait=True)
        graber.run_target(100, open_grb, then=Stop.HOLD, wait=True)
        height.run_target(100, up_2, then=Stop.HOLD, wait=True)
    elif level == 2:
        height.run_target(100, up_1,then=Stop.HOLD, wait=True)
        graber.run_target(100, open_grb,then=Stop.HOLD, wait=True)
        height.run_target(100, up_2,then=Stop.HOLD, wait=True)

def pipe():
    height.run_target(100, down, then=Stop.HOLD, wait=True)
    graber.run_target(100, open_grb_full, then=Stop.HOLD, wait=True)
    height.run_target(100, up_2, then=Stop.HOLD, wait=True)

def open():
    height.run_target(100, up_3,then=Stop.HOLD, wait=True)

def close():
    height.run_target(100, up_1,then=Stop.HOLD, wait=True)

def gyro_turn(deg):
    gyro.reset_angle(0)
    if deg > 0:
        dire = 1
    elif deg < 0:
        dire = -1

    while gyro.angle() != deg:
        mnr.drive(100,1000 * dire)
    
    mnr.stop()
