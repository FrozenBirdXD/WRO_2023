from pybricks.ev3devices import Motor
from pybricks.parameters import Stop, Direction, Color
from pybricks.tools import wait
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

    def drive(self, speed, turn_rate, time):
        left_speed = speed
        right_speed = speed
        if turn_rate > 0:
            right_speed *= (1000 - (turn_rate * 2)) / 1000
        elif turn_rate < 0:
            left_speed *= (1000 + (turn_rate * 2)) / 1000
        
        self.left_motor.run(left_speed)
        self.right_motor.run(-right_speed)
        wait(time)
        self.stop()

    def drive_until(self, speed, color:Color):
        while self.left_color_sensor.color is not color or self.right_color_sensor.color is not color:
            self.left_motor.run(speed)
            self.right_motor.run(-speed)
        self.left_motor.hold()
        self.right_motor.hold()

    def line_tracer(self, side: str, direction:str):
        if (direction == "back"):
            speed = -500
        else:
            speed = 500

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

    # def gyro_turn(self, deg):
    #     self.gyro_sensor.reset_angle(0)
    #     if deg > 0:
    #         dire = 1
    #     elif deg < 0:
    #         dire = -1

    #     while abs(self.gyro_sensor.angle()) < abs(deg * 0.964):
    #         self.drive(200, 1000 * dire)

    #     self.stop()
    
    # constants
    WHEEL_DIAMETER = 58  # in millimeters
    WHEEL_BASE_WIDTH = 161.5 #161
