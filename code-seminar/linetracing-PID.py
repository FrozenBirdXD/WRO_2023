import color_sensor
from hub import light_matrix, port
import motor_pair
import runloop

motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)

async def main():
    target = 60 # on line value    (black + white) / 2
    integral = 0
    last_error = 0

    kp = 1.95 # too violent decrease, cannot follow increase
    ki = 0.00
    kd = 0.15

    # speed -660 - 660

    while (True):
        value = color_sensor.reflection(port.D)
        error = target - value

        proportional = error
        integral += error
        derivative = error - last_error

        pid = proportional * kp + integral * ki + derivative * kd

        motor_pair.move(motor_pair.PAIR_1, int(pid), velocity = 330)

        last_error = error


runloop.run(main())
