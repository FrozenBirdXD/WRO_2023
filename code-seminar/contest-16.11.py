from hub import light_matrix, port, motion_sensor
import motor, color_sensor, motor_pair
import runloop, math, time

motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)

async def main():
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1700, 0, velocity=-2000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -180, 100, velocity=-2000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 3400, 0, velocity=-2000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -125, 100, velocity=-2000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1750, 0, velocity=-2000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -215, 100, velocity=-2000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 2150, 0, velocity=-2000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -25, 100, velocity=-2000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 2700, 0, velocity=-2000) 
    
runloop.run(main())
