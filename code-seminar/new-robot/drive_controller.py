from pybricks.parameters import Stop
from pybricks.ev3devices import Motor


# Drivebase
class DriveController:
    def __init__(self, left_motor: Motor, right_motor: Motor, wheel_diameter):
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

    def drive_distance(self, speed, distance_mm):
        # Calculate number of degrees to rotate
        circumference = self.wheel_diameter * 3.1415926
        angle_degrees = (distance_mm / circumference) * 360

        self.left_motor.run_angle(speed, angle_degrees, then=Stop.HOLD, wait=False)
        self.right_motor.run_angle(-speed, angle_degrees, then=Stop.HOLD, wait=False)

    # constants
    WHEEL_DIAMETER = 62.4  # in millimeters
