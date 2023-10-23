from hub import light_matrix, port
import motor, color_sensor, motor_pair
import runloop, math, time

async def main():
    black = 22
    white = 90
    mw = (white + black) / 2

    color_sensor.reflection(port.D)
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
    await runloop.sleep_ms(2)
    start_time = time.ticks_ms()
    while time.ticks_ms() - start_time < 50000:
        motor_pair.move(motor_pair.PAIR_1, int(((color_sensor.reflection(port.D) - mw) * -0.11) ** 3), velocity = 500)
    motor_pair.stop(motor_pair.PAIR_1)

runloop.run(main())
