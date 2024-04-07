from pybricks.ev3devices import Motor
from pybricks.parameters import Stop, Direction, Color
from pybricks.tools import wait
from math import sqrt,sin,cos,tan,sinh,cosh,tanh
from pybricks.ev3devices import (
    Motor,
    ColorSensor,
    GyroSensor,
)

# Constants
Kp = 10  # can be larger
Ki = 0.0000  # small
Kd = 0.0  # small
TARGET_REFLECTION = 27

# Drivebase
class DriveController:
    def __init__(self, left_motor: Motor, right_motor: Motor, wheel_diameter:int, left_color_sensor: ColorSensor, right_color_sensor: ColorSensor):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_diameter = wheel_diameter
        self.left_color_sensor = left_color_sensor
        self.right_color_sensor = right_color_sensor

    def drive(self, speed, turn_rate):
        left_speed = speed
        right_speed = speed
        if turn_rate > 0:
            right_speed *= (1000 - (turn_rate * 2)) / 1000
        elif turn_rate < 0:
            left_speed *= (1000 + (turn_rate * 2)) / 1000
        
        self.left_motor.run(left_speed)
        self.right_motor.run(-right_speed)

    def drive_until(self, speed, color:Color):
        while self.left_color_sensor.color() is not color and self.right_color_sensor.color() is not color:
            print(self.left_color_sensor.color())
            print(self.right_color_sensor.color())
            self.left_motor.run(speed)
            self.right_motor.run(-speed)
        self.left_motor.hold()
        self.right_motor.hold()

    def line_tracer(self, side: str, direction:str):
        if (direction == "back"):
            speed = -300
        else:
            speed = 300

        last_error = 0

        target = TARGET_REFLECTION
        integral = 0

        if side == "right":
            value = self.right_color_sensor.reflection()
            multiplier = 1
        elif side == "left":
            value = self.left_color_sensor.reflection()
            multiplier = -1
        else:
            raise ValueError("Invalid side")

        error = target - value
        integral += error
        derative = error - last_error

        correction = (error * Kp) + (integral * Ki) + (derative * Kd)

        self.drive(speed, correction * multiplier)

        last_error = error

    
    def straighten(self, direction:str):
        if direction == "front":
            K = 3
        elif direction == "back":
            K = -3
        left = self.left_color_sensor.reflection()
        right = self.right_color_sensor.reflection()
        while True:
            left = self.left_color_sensor.reflection() - 3
            error = TARGET_REFLECTION - left
            speed_left = (error * K)
            right = self.right_color_sensor.reflection()
            error = TARGET_REFLECTION - right
            speed_right = (error * K)
            if left == right == TARGET_REFLECTION:
                self.stop()
                break
            self.left_motor.run(-speed_left)
            self.right_motor.run(speed_right)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def drive_distance(self, speed, distance_mm):
        # Calculate number of degrees to rotate
        circumference = self.wheel_diameter * 3.14
        angle_degrees = (distance_mm / circumference) * 360

        self.left_motor.run_angle(speed, angle_degrees, then=Stop.HOLD, wait=False)
        self.right_motor.run_angle(-speed, angle_degrees, then=Stop.HOLD, wait=True)
        
    def turn(self, speed, angle_degrees):
        # Calculate distance
        wheel_travel_distance = (angle_degrees * 2 / 360) * 3.141 * self.WHEEL_BASE_WIDTH

        self.left_motor.run_angle(speed, wheel_travel_distance, then=Stop.HOLD, wait=False)
        self.right_motor.run_angle(speed, wheel_travel_distance, then=Stop.HOLD, wait=True)

    def u_trun(self,speed,x,factor):
        r_main = abs(x)/2
        r_big= r_main + 0.5*self.WHEEL_BASE_WIDTH
        r_small = r_main - 0.5*self.WHEEL_BASE_WIDTH
        motor_angle_big = ((3.141*r_big*2*factor)/(self.WHEEL_DIAMETER*3.141))*402
        motor_angle_small = (((3.141*r_small*2*factor))/(self.WHEEL_DIAMETER*3.141))*402
        turn_rate = (motor_angle_small/motor_angle_big)
        if x>0:
            self.__curve_u_turn(speed,turn_rate,1,motor_angle_small,motor_angle_big)
        elif x<0:
            self.__curve_u_turn(speed,1,turn_rate,motor_angle_big,motor_angle_small)


    def __curve_u_turn(self,speed, right_turn_rate,left_turn_rate, right_deg, left_deg):
        left_speed = speed
        right_speed = speed
        
        right_speed *= right_turn_rate
        left_speed *= left_turn_rate

        self.left_motor.run_angle(left_speed,left_deg,then=Stop.HOLD,wait=False)
        self.right_motor.run_angle(-right_speed,right_deg,then=Stop.HOLD,wait=True)


    def __curve_shift(self,speed, right_turn_rate,left_turn_rate, right_deg, left_deg):
        left_speed = speed
        right_speed = speed
        
        right_speed *= right_turn_rate
        left_speed *= left_turn_rate

        if right_deg > left_deg:
            self.left_motor.run_angle(left_speed,left_deg,then=Stop.HOLD,wait=False)
            self.right_motor.run_angle(-right_speed,right_deg,then=Stop.HOLD,wait=True)
        else:
            self.right_motor.run_angle(-right_speed,right_deg,then=Stop.HOLD,wait=False)
            self.left_motor.run_angle(left_speed,left_deg,then=Stop.HOLD,wait=True)
            

    def shift(self,x,y,speed):
        a = 0.5 * sqrt(y*y + x*x)
        beta = sinh(abs(y)/(2*a))
        alpha = (3.141)-2*beta
        factor = (alpha/(2*3.141))
        b = (a/2)/cos(beta)
    
        r_main = b
        r_big= r_main + 0.5*self.WHEEL_BASE_WIDTH
        r_small = r_main - 0.5*self.WHEEL_BASE_WIDTH
        motor_angle_big = (((3.141*r_big*2)*factor)/(self.WHEEL_DIAMETER*3.141))*365
        motor_angle_small = (((3.141*r_small*2)*factor)/(self.WHEEL_DIAMETER*3.141))*365
        turn_rate = (motor_angle_small/motor_angle_big)

        if x>0 and y>0:
            self.__curve_shift(speed,turn_rate,1,motor_angle_small,motor_angle_big)
            self.__curve_shift(speed,1,turn_rate,motor_angle_big,motor_angle_small)
        elif x<0 and y<0:
            self.__curve_shift(-speed,1,turn_rate,motor_angle_big,motor_angle_small)
            self.__curve_shift(-speed,turn_rate,1,motor_angle_small,motor_angle_big)
        elif x>0 and y<0:
            self.__curve_shift(-speed,turn_rate,1,motor_angle_small,motor_angle_big)
            self.__curve_shift(-speed,1,turn_rate,motor_angle_big,motor_angle_small)
        elif x<0 and y>0:
            self.__curve_shift(speed,1,turn_rate,motor_angle_big,motor_angle_small)
            self.__curve_shift(speed,turn_rate,1,motor_angle_small,motor_angle_big)


    def turn_after(self,x,y,direction,speed=1000):
        self.drive_distance(speed,y)
        if direction == "left":
            self.u_trun(speed-200,-200,0.25)
        else:
            self.u_trun(speed-200,200,0.25)
        self.drive_distance(speed,x)


    
    # constants
    WHEEL_DIAMETER = 62  # in millimeters
    WHEEL_BASE_WIDTH = 161.5 #161
