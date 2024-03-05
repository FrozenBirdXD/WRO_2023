from hub import light_matrix, port, motion_sensor
import motor, color_sensor, motor_pair
import runloop, math, time

async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motor_pair.move(motor_pair.PAIR_1, 0,velocity = 500)
    while motion_sensor.tilt_angles()[0] < 90:
        motor_pair.move(motor_pair.PAIR_1, -100, velocity = 500)
    
runloop.run(main())
