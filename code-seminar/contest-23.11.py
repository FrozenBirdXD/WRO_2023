import color_sensor
from hub import light_matrix, port
import motor
import motor_pair
import runloop

motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)

def line_tracer(place: str, speed: int):
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
    ki = 0.000
    kd = 0

    # speed -660 - 660

    # while (True):
    value = color_sensor.reflection(ports)
    error = target - value
    proportional = error
    integral += error
    derivative = error - last_error
    pid = proportional * kp + integral * ki + derivative * kd
    motor_pair.move(motor_pair.PAIR_1, int(x * pid), velocity = speed)
    last_error = error

# turn for an amount (steering -100 to 100)
async def turn(steering: int, degrees: int):
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees, steering, velocity = 100)

# move 120 degrees forward
async def move_before_turn():
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 120, 0, velocity = 200)

async def move_until_left():
    while color_sensor.reflection(port.D) > 22: # follow line until left sensor reads black
        line_tracer("right", 200)
    await move_before_turn()                    # move a bit forward to compensate for turn

async def move_until_right():
    while color_sensor.reflection(port.F) > 22:
        line_tracer("left", 200)
    await move_before_turn()

def move_until_col(color: int):
    colors = []
    while color_sensor.color(port.D) != color:
        # print(color_sensor.color(port.D))
        if color_sensor.color(port.D) not in colors:
            colors.append(color_sensor.color(port.D))
        line_tracer("right", 200)
    return colors

async def pic_up(v):
    await motor.run_for_degrees(port.C,v * -100, 200)

async def take_away(color: int):
    await turn(-100, 370)

    #await motor_pair.move_for_degrees(motor_pair.PAIR_1, -400, 0, velocity = 400)

    while color_sensor.color(port.F) != color:
        motor_pair.move(motor_pair.PAIR_1, 0, velocity = 400)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -50, 0, velocity = 100)


async def main():
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 300, 0, velocity = 300)
    colors = move_until_col(10)
    colors.remove(-1)
    print(colors)
    await move_before_turn()
    await turn(-100, 180)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 100, 0, velocity = 500)
    await pic_up(1)
    await take_away(colors[0])
    await pic_up(-1)

runloop.run(main())
