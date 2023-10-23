from hub import light_matrix, port, motion_sensor
import motor, color_sensor, motor_pair
import runloop, math, time

motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)

def line_tracer():
    if color_sensor.reflection(port.D) < 80:
        motor_pair.move(motor_pair.PAIR_1, -int(((color_sensor.reflection(port.D) - 90) / 7) ** 2), velocity = 500)
    elif color_sensor.reflection(port.F) < 80:
        motor_pair.move(motor_pair.PAIR_1, int(((color_sensor.reflection(port.F) - 90) / 7) ** 2), velocity = 500)


def turn_left():
    while True:
        line_tracer()
        if color_sensor.reflection(port.D) < 22:
            motor_pair.move_for_degrees(motor_pair.PAIR_1, 180, -100, velocity = 100)
        break
        
def turn_right():
    while True:
        line_tracer()
        if color_sensor.reflection(port.F) < 30:
            motor_pair.move(motor_pair.PAIR_1, 100, velocity = 500)
            break

async def main():
    turn_left()

runloop.run(main())
