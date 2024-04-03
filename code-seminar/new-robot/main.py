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
left_motor = Motor(Port.A, positive_direction = Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
graber_motor = Motor(Port.D)
height_motor = Motor(Port.C)
# Limits
left_motor.control.limits(1500, 2500, 100)
right_motor.control.limits(1500, 2500, 100)
graber_motor.control.limits(1500, 4000, 100)
height_motor.control.limits(1500, 1500, 100)

# Sensors
left_color_sensor = ColorSensor(Port.S4)
right_color_sensor = ColorSensor(Port.S2)
main_color_sensor = ColorSensor(Port.S3)

# ------------------------------------------------
# just initialize global objects

mnr = DriveController(left_motor, right_motor, DriveController.WHEEL_DIAMETER, left_color_sensor, right_color_sensor)
graber = Graber(height_motor, graber_motor)

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

def picup_r_y():
    #straighten
    mnr.drive_distance(300,-90)
    mnr.drive_distance(200,80)
    wait(100)
    #drive to first block
    mnr.drive_distance(400, 190)
    mnr.turn(300, 90)
    mnr.drive_distance(400, 240)
    mnr.turn(300, 90)
    # mnr.straighten("front")
    mnr.drive_distance(400, 120)

    #grab first block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    #drive to second block
    mnr.drive_distance(300, -70)
    mnr.turn(250, -90)
    mnr.drive_distance(300, 95)
    mnr.turn(250, 90)
    # mnr.straighten("front")
    mnr.drive_distance(300, 75)

    #grab second block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    
    #grab first stack of blocks
    graber.graber_ready()
    graber.height_up_aggressive()
    mnr.turn(200, 20)
    right_motor.run_angle(speed=200, rotation_angle=20, then=Stop.HOLD, wait=False)
    mnr.drive_distance(200, 110)

    #drive to thrid block
    graber.height_carry()
    mnr.drive_distance(300, -165)
    mnr.turn(200, -20)

    wait(100)

    mnr.turn(200, -90)
    mnr.drive_distance(300, 80)
    mnr.turn(200, 91)

    # mnr.straighten("back")
    mnr.drive_distance(300, 60)

    #grab third block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    #drive to fourth block
    mnr.drive_distance(300, -70)
    mnr.turn(200, -90)
    mnr.drive_distance(300, 90)
    mnr.turn(200, 91)
    # mnr.straighten("back")
    mnr.drive_distance(300, 75)

    #grab fourth block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    #put blocks on color
    graber.height_carry()

def picup_r_y_2():
    #go to first block
    mnr.drive_distance(800,-90)
    mnr.drive_distance(1000,120)
    mnr.u_trun(600,240,0.5)

    #grab first block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    #shift to next block
    mnr.shift(-105,-150,1500)
    mnr.drive_distance(1500, 150)

    #grab second block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    
    #grab first stack of blocks
    graber.graber_ready()
    graber.height_up_aggressive()
    mnr.turn(200, 20)
    right_motor.run_angle(speed=200, rotation_angle=20, then=Stop.HOLD, wait=False)
    mnr.drive_distance(300, 110)
    graber.height_carry()
    mnr.drive_distance(700, -110)
    mnr.turn(200, -20)


    mnr.shift(-105,-150,1500)
    mnr.drive_distance(1500, 150)

    #grab third block
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()
    graber.height_carry()

    #drive to fourth block
    mnr.shift(-105,-150,1500)
    mnr.drive_distance(1500, 150)

    #grab fourth block
    graber.height_2()
    graber.graber_ready()
    graber.height_1()
    graber.graber_close()

    #put blocks on color
    graber.height_carry()

def put_back_1():
    #--------------------------------------new straighten with border
    mnr.u_trun(-800,200,0.5)
    mnr.drive_distance(500, -210)

    #--------------------------------------new 
    mnr.drive_distance(1000,340)
    mnr.u_trun(600,-280,0.25)
    graber.height_up()
    mnr.drive_distance(1000, -295)
    graber.height_1()
    graber.graber_open()
    graber.height_up_aggressive()

def rth():
    mnr.drive_distance(1000,-220)
    mnr.u_trun(-600, 300, 0.25)
    mnr.drive_distance(1000, -130)

def put_back_2():
    mnr.drive_distance(250, -160)
    mnr.drive_distance(250, 100)
    mnr.turn(300, 90)
    mnr.drive_distance(250, 20)
    mnr.turn(300, 90)
    mnr.drive_distance(250, 90)

    graber.height_4()
    graber.graber_ready()
    graber.height_up()

    mnr.drive_distance(250, -100)
    
if __name__ == "__main__":
    picup_r_y_2() 
    put_back_1()
    rth()
    picup_r_y_2()
    put_back_2()

    
    