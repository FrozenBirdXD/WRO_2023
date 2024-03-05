from pybricks.parameters import Stop
from pybricks.ev3devices import Motor


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
        graber_motor: Motor,
        height_motor: Motor,
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

    # Constants
    OPEN_GRABBER_FULL = 180
    OPEN_GRABBER = 0
    CLOSE_GRABBER = 90
    HEIGHT_DOWN = 0
    HEIGHT_UP_1 = 40
    HEIGHT_UP_2 = 80
    HEIGHT_UP_3 = 180
