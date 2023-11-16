import color_sensor
from hub import light_matrix, port
import motor_pair
import runloop

motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)

async def line_tracer(place:str):
    if place == "left":
        ports = port.D
        x = -1
    else:
        ports = port.F
        x = 1
    target = 60 # on line value    (black + white) / 2
    integral = 0
    last_error = 0

    kp = 0.8 # too violent decrease, cannot follow increase
    ki = 0.00
    kd = 0.15

    # speed -660 - 660
    
    # while (True):
    value = color_sensor.reflection(ports)
    error = target - value
    proportional = error
    integral += error
    derivative = error - last_error
    pid = proportional * kp + integral * ki + derivative * kd
    motor_pair.move(motor_pair.PAIR_1, int(x*pid), velocity = 600)
    last_error = error

# turn for an amount (steering -100 to 100)
async def turn(steering: int,degrees: int):
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees, steering, velocity = 100)

# move 120 degrees forward
async def move_before_turn():
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 100, 0, velocity = 300)

async def move_until_left():
    while color_sensor.reflection(port.D) > 22: # follow line until left sensor reads black
        await line_tracer("right")
    await move_before_turn()                    # move a bit forward to compensate for turn

async def move_until_right():
    while color_sensor.reflection(port.F) > 22:
        await line_tracer("left")
    await move_before_turn()


async def main():
    #deutsch()
    await move_until_left()
    await turn(-100,240)            # turn left
    await move_until_right()
    await turn(100,240)            # turn right
    await move_until_left()
    await turn(-100,240)            # turn left
    await move_until_right()
    await turn(100,240)
    while True:
        await line_tracer("left")

runloop.run(main())
