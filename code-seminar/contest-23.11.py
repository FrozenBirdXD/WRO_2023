import color_sensor
from hub import light_matrix, port,motion_sensor
import motor
import motor_pair
import runloop

motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)

def line_tracer(place: str, speed: int):
    """Trace a black line, with one iteration of the pid programm. Continuous loop is needed"""
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

async def turn(steering: int, degrees: int):
    """Turn for a specific amount in degrees (steering -100 to 100)"""
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees, steering, velocity = 100)

async def move_before_turn():
    """Move 120 degrees forward to compensate for turn"""
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 120, 0, velocity = 200)

async def move_until_left():
    """Follow black line until left sensor reads black"""
    while color_sensor.reflection(port.D) > 22:
        line_tracer("right", 200)
    await move_before_turn()                  

async def move_until_right():
    """Follow black line until right sensor reads black"""
    while color_sensor.reflection(port.F) > 22:
        line_tracer("left", 200)
    await move_before_turn()

def move_until_col(color: int) -> list[int]:
    """Follows black line until color sensor on the left senses the provided color (Black = 0, Magenta = 1, Purple = 2, Blue = 3, Azure = 4, Turquoise = 5, Green = 6, Yellow = 7, Orange = 8, Red = 9, White = 10, Unknown = -1)\n\nReturns list of sensed colors"""
    colors = []
    while color_sensor.color(port.D) != color:
        if color_sensor.color(port.D) not in colors:
            colors.append(color_sensor.color(port.D))
        line_tracer("right", 200)
    return colors

async def lower_grabber():
    await motor.run_for_degrees(port.C, -100, 200)

async def raise_grabber():
    await motor.run_for_degrees(port.C, -1 * -100, 200)

async def turn_block_into_center():
    """Turns the block into the center of the circle and backs up\n\nUses gyro to measure the amount turned"""
    motion_sensor.reset_yaw(0)
    while color_sensor.color(port.F) != 10:
        motor_pair.move(motor_pair.PAIR_1, 100, velocity = 100)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 0, 0, velocity = 100)

    await raise_grabber()

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -50, 0, velocity = 100)
    while motion_sensor.tilt_angles()[0] <= 0:
        motor_pair.move(motor_pair.PAIR_1, -100, velocity = 100)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 0, 0, velocity = 100)

async def take_away(color: int):
    """Drives block into the circle with the provided color (Black = 0, Magenta = 1, Purple = 2, Blue = 3, Azure = 4, Turquoise = 5, Green = 6, Yellow = 7, Orange = 8, Red = 9, White = 10, Unknown = -1)"""
    await turn(-100, 365) # turn almost 180°

    while color_sensor.color(port.F) != color: # drives forward until certain color
        motor_pair.move(motor_pair.PAIR_1, 0, velocity = 500)
    
    await turn_block_into_center();

async def drive_home(colors):
    """Drives the robot into the home corner of the provided color from the circles in the center of the board"""
    color = colors[1]
    if color == 7: # yellow
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, -50, 0, velocity = 200)
        await turn(-100, 185)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 420, 0, velocity = 800)
        await turn(-100, 175)
        while color_sensor.color(port.F) != color:
            motor_pair.move(motor_pair.PAIR_1, 0, velocity = 800)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 500, 0, velocity = 300)
        await lower_grabber()
        

        # await turn(-100, 185)
        # await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1800, 0, velocity = 800)
    elif color == 6: # green
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, -50, 0, velocity = 600)
        await turn(-100, 185)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 420, 0, velocity = 600)
        await turn(100, 185)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1900, 0, velocity = 900)
        await lower_grabber()

        # await turn(-100, 368)
        # while color_sensor.color(port.F) != 7:
        #     motor_pair.move(motor_pair.PAIR_1, 0, velocity = 800)
        # await motor_pair.move_for_degrees(motor_pair.PAIR_1, 600, 0, velocity = 300)

        # await turn(-100, 185)
        # await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1800, 0, velocity = 800)
    elif color == 9: # red
        if colors[0] != 6:
            while color_sensor.color(port.F) != 6:
                motor_pair.move(motor_pair.PAIR_1, 0, velocity = -400)

        await motor_pair.move_for_degrees(motor_pair.PAIR_1, -50, 0, velocity = 200)
        await turn(100, 185)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 200, 0, velocity = 800)
        while color_sensor.color(port.F) != color:
            motor_pair.move(motor_pair.PAIR_1, 0, velocity = 500)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 150, 0, velocity = 400)

        await turn(-100,176)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 900, 0, velocity = 400)
        await lower_grabber()

        # await turn(100, 180)
        # await motor_pair.move_for_degrees(motor_pair.PAIR_1, 2200, 0, velocity = 800)
    elif color == 3: # blue
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, -100, 0, velocity = 200)
        await turn(100, 185)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1150, 0, velocity = 800)
        await turn(-100, 179)
        while color_sensor.color(port.F) != color:
            line_tracer("right",400)
            # motor_pair.move(motor_pair.PAIR_1, 0, velocity = 800)
        await turn(100,30)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, 80, 0, velocity = 200)
        await lower_grabber()
        # await turn(-100, 368)
        # await motor_pair.move_for_degrees(motor_pair.PAIR_1, 4300, 0, velocity = 800)

async def drive_from_to_color(colors):
    colorakt = colors[1]
    colorgo = colors[2]
    if colorakt == 9:
        await turn(-100,10)
        if colorgo == 3:
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)

            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
        elif colorgo == 6:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1000, 0, velocity = 800)
            await turn(-100,185)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)

            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)

        elif colorgo == 7:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1000, 0, velocity = 800)
            await turn(-100,185)
            while color_sensor.color(port.F) != 6:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await turn(-100,185)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)

            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
    elif colorakt == 3:
        if colorgo == 9:
            await turn(100,340)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,2000,0,velocity=800)
            await turn(-100,180)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
        elif colorgo == 6:
            await turn(-100,240)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,0,velocity=300)
        elif colorgo == 7:
            await turn(-100,240)
            while color_sensor.color(port.F) != 6:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
            await turn(-100,140)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
    elif colorakt == 6:
        if colorgo == 3:
            await turn(100,185)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,1700,0,velocity=800)
            await turn(-100,185)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
        elif colorgo == 9:
            await turn(100,185)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,1900,0,velocity=800)
            await turn(100,185)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
        elif colorgo == 7:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,-3500,0,velocity=800)
    elif colorakt == 7:
        if colorgo == 6:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,-3500,0,velocity=800)
            # await turn(100,185)
            # while color_sensor.color(port.F) != colorgo:
            #     motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            # await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
        elif colorgo == 3:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,-3500,0,velocity=800)
            await turn(-100,250)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)
        elif colorgo == 9:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,-3500,0,velocity=800)
            await turn(-100,220)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,1400,0,velocity=800)
            await turn(100,180)
            while color_sensor.color(port.F) != colorgo:
                motor_pair.move(motor_pair.PAIR_1,0,velocity=800)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,0,0,velocity=0)


def remove_invalid_colors(colors: list): 
    """Removes impossible colors from list"""
    colors_to_remove = [-1, 0, 1, 2, 4, 5, 8, 10] # List Comprehension: faster than if statements, since optimized with c and 
    colors = [color for color in colors if color not in colors_to_remove]
    return colors

async def main():
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 300, 0, velocity = 300) # move to find line
    colors = move_until_col(10) # read block colors until block is white (10)
    colors = remove_invalid_colors(colors) # remove impossible colors in list
   
    print(colors)

    # turn
    await move_before_turn()
    await turn(-100, 180)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 100, 0, velocity = 500)

    await lower_grabber()
    await take_away(colors[0]) # takes first element in array
    await drive_home(colors)
    await drive_from_to_color(colors)

runloop.run(main())
