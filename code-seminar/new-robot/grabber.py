from pybricks.parameters import Stop
from pybricks.ev3devices import Motor


class Grabber:
    def __init__(self, height_motor:Motor, graber_motor:Motor):
        self.height_motor = height_motor
        self.graber_motor = graber_motor
        self.height_motor.reset_angle(0)
        self.graber_motor.reset_angle(0)

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

    def open_pipe(self):
        self.height_motor.run_target(100, self.height_down, then=Stop.HOLD, wait=True)
        self.graber_motor.run_target(
            100, self.grab_open_full, then=Stop.HOLD, wait=True
        )
        self.height_motor.run_target(100, self.height_up_2, then=Stop.HOLD, wait=True)

    def nach_hinten_digga(self):
        self.height_motor.run_angle(300, 100, then=Stop.HOLD, wait=True)

    def graber_close(self):
        self.graber_motor.run_target(800, self.CLOSE_GRABER, then=Stop.HOLD, wait=True)

    def graber_open(self):
        self.graber_motor.run_target(800, self.OPEN_GRABER, then=Stop.HOLD, wait=True)

    def graber_open_full(self):
        self.graber_motor.run_target(800, self.OPEN_GRABER_FULL, then=Stop.HOLD, wait=True)

    def height_down(self):
        self.height_motor.run_target(800, self.HEIGHT_1, then=Stop.HOLD, wait=True)

    def height_up(self):
        self.height_motor.run_target(800, self.HEIGHT_UP, then=Stop.HOLD, wait=True) 

    def height_4(self):
        self.height_motor.run_target(800, self.HEIGHT_4, then=Stop.HOLD, wait=True)

    def height_2(self):
        self.height_motor.run_target(800, self.HEIGHT_2, then=Stop.HOLD, wait=True)

    def height_carry(self):
        self.height_motor.run_target(800, self.HEIGHT_CARRY, then=Stop.HOLD, wait=True)

    # Constants
    OPEN_GRABER_FULL = 0
    OPEN_GRABER = 250
    CLOSE_GRABER = -580
    HEIGHT_UP = 0
    HEIGHT_1 = -670
    HEIGHT_2 = -600
    HEIGHT_4 = -520
    HEIGHT_CARRY = -450

