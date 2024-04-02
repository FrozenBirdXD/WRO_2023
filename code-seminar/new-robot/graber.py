from pybricks.parameters import Stop
from pybricks.ev3devices import Motor


class Graber:
    def __init__(self, height_motor:Motor, graber_motor:Motor):
        self.height_motor = height_motor
        self.graber_motor = graber_motor
        self.height_motor.reset_angle(0)
        self.graber_motor.reset_angle(0)

    def graber_close(self):
        self.graber_motor.run_target(800, self.CLOSE_GRABER, then=Stop.HOLD, wait=True)

    def graber_open(self):
        self.graber_motor.run_target(800, self.OPEN_GRABER, then=Stop.HOLD, wait=True)

    def graber_open_full(self):
        self.graber_motor.run_target(800, self.OPEN_GRABER_FULL, then=Stop.HOLD, wait=True)

    def graber_ready(self):
        self.graber_motor.run_target(800, self.READY_GRABER, then=Stop.HOLD, wait=True)

    def height_1(self):
        self.height_motor.run_target(800, self.HEIGHT_1, then=Stop.HOLD, wait=True)

    def height_up(self):
        self.height_motor.run_target(400, self.HEIGHT_UP, then=Stop.HOLD, wait=True) 

    def height_up_aggressive(self):
        self.height_motor.run_target(1000, self.HEIGHT_UP, then=Stop.HOLD, wait=True) 

    def height_4(self):
        self.height_motor.run_target(800, self.HEIGHT_4, then=Stop.HOLD, wait=True)

    def height_2(self):
        self.height_motor.run_target(800, self.HEIGHT_2, then=Stop.HOLD, wait=True)

    def height_carry(self):
        self.height_motor.run_target(800, self.HEIGHT_CARRY, then=Stop.HOLD, wait=True)
    
    def height_complete_breakdown(self):
        self.height_motor.run_target(800, -900, then=Stop.HOLD, wait=True)
    
    def yalla(self):
        self.height_motor.run_target(1500, -500, then=Stop.HOLD, wait=True)
    
    def graber_open_for_pipe_because_just_because(self):
        self.graber_motor.run_target(800, -300, then=Stop.HOLD, wait=True)

    # Constants
    OPEN_GRABER_FULL = 0
    READY_GRABER = -460 #-490
    OPEN_GRABER = -250 
    CLOSE_GRABER = -590 
    HEIGHT_UP = 10 
    HEIGHT_1 = -780 #-753
    HEIGHT_2 = -710 #-683
    HEIGHT_4 = -603
    HEIGHT_CARRY = -450

