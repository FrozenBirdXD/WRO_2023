from hub import light_matrix, port, motion_sensor
import motor, color_sensor, motor_pair
import runloop, math, time

motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)

# single condition for line tracing (without loop)
def line_tracer():
    if color_sensor.reflection(port.D) < 80:
        motor_pair.move(motor_pair.PAIR_1, -int(((color_sensor.reflection(port.D) - 90) / 8) ** 2), velocity = 400)
    elif color_sensor.reflection(port.F) < 80:
        motor_pair.move(motor_pair.PAIR_1, int(((color_sensor.reflection(port.F) - 90) / 8) ** 2), velocity = 400)
    else:
        motor_pair.move(motor_pair.PAIR_1, 0, velocity = 500)

# turn for an amount (steering -100 to 100)
async def turn(steering: int):
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 180, steering, velocity = 100)

# move 120 degrees forward
async def move_before_turn():
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 120, 0, velocity = 300)

async def move_until_left():
    while color_sensor.reflection(port.D) > 22: # follow line until left sensor reads black
        line_tracer()
    await move_before_turn()                    # move a bit forward to compensate for turn

async def move_until_right():
    while color_sensor.reflection(port.F) > 22:
        line_tracer()
    await move_before_turn()

def deutsch():
    motor.run(port.C, 500)

async def main():
    deutsch()
    await move_until_left()
    await turn(-100)            # turn left
    await move_until_right()
    await turn(100)             # turn right
    while True:
        line_tracer()

runloop.run(main())
