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
right_motor = Motor(Port.B)
graber = Motor(Port.D)
height = Motor(Port.C)
left_motor.control.limits(1500,1000000000,8000)
right_motor.control.limits(1500,1000000000,8000)
graber.control.limits(1500,1000000000,8000)
height.control.limits(1500,1000000000,8000)

#Drivebase
mnr = DriveBase(left_motor,right_motor,axle_track = 130,wheel_diameter = 70)

#Sensors
b_l_color = ColorSensor(Port.S4)
b_r_color = ColorSensor(Port.S2)
main_color = ColorSensor(Port.S3)
gyro = GyroSensor(Port.S1)


#testing








# Write your program here.
class Adjusting:
    def __init__(self):
        pass

    def line_tracer(self,side:str):
        target = 30
        integral = 0
        error = 0
        last_error = 0
        derative = 0

        Kp = 1 # can be larger
        Ki = 0.0001 # needs to be quite small
        Kd = 0.1 # needs to be small too

        if side == "right":
            value = b_r_color.reflection()
            dir = -1
        elif side == "left":
            value = b_l_color.reflection()
            dir = 1

        error = target - value
        integral += error
        derative = error - last_error

        correction = (error * Kp)+(integral * Ki)+(derative * Kd)
        mnr.drive(200,correction * dir)
        
        last_error = error

    def straighten(self):
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

class Graber:
    open_grb_full = 180
    open_grb = 0
    close_grb = 90

    up_1 = 40
    up_2 = 80
    up_3 = 180
    down = 0
    def __init__(self):
        pass

    def picup(self,level):
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

    def drop(self,level):
        if level == 1:
            height.run_target(100, down, then=Stop.HOLD, wait=True)
            graber.run_target(100, open_grb, then=Stop.HOLD, wait=True)
            height.run_target(100, up_2, then=Stop.HOLD, wait=True)
        elif level == 2:
            height.run_target(100, up_1,then=Stop.HOLD, wait=True)
            graber.run_target(100, open_grb,then=Stop.HOLD, wait=True)
            height.run_target(100, up_2,then=Stop.HOLD, wait=True)
    
    def pipe(self):
        height.run_target(100, down, then=Stop.HOLD, wait=True)
        graber.run_target(100, open_grb_full, then=Stop.HOLD, wait=True)
        height.run_target(100, up_2, then=Stop.HOLD, wait=True)
    
    def open(self):
        height.run_target(100, up_3,then=Stop.HOLD, wait=True)

    def close(self):
        height.run_target(100, up_1,then=Stop.HOLD, wait=True)

def gyro_turn(deg):
    gyro.reset_angle(0)
    if deg > 0:
        dir = 1
    elif deg < 0:
        dir = -1

    while gyro.angle() != deg:
        mnr.drive(50,100 * dir)
    
    mnr.stop()

right_motor.run(1500)
left_motor.run(-1500)
wait(3000)
