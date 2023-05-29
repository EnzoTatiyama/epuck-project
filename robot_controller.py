from controller import Robot, Supervisor, LED

TIMESTEP = 32
MAX_SPEED = 6.28
MIN_DISTANCE = 0.08
SENSOR_DISTANCE = 150

def main():
    robot = Robot()
    supervisor = Supervisor()
    
    leds = []
    for i in range(10):
        leds.append(robot.getDevice('led' + str(i)))
        leds[i].set(0)

    left_motor = robot.getDevice('left wheel motor')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor = robot.getDevice('right wheel motor')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    right_front_distance_sensor = robot.getDevice('ps0')
    right_front_distance_sensor.enable(TIMESTEP)
    
    left_front_distance_sensor = robot.getDevice('ps7')
    left_front_distance_sensor.enable(TIMESTEP)

    epuck_position_translation = supervisor.getFromDef("EPUCK").getField("translation")
    box_position_translation = supervisor.getFromDef("WOODENBOX").getField("translation")

    while robot.step(TIMESTEP) != -1:
        # wooden box coordinates
        new_box_position = box_position_translation.getSFVec3f()
        x_box_current = new_box_position[0]
        y_box_current = new_box_position[1]

        # e-puck coordinates
        new_epuck_position = epuck_position_translation.getSFVec3f()
        x_epuck_current = new_epuck_position[0]
        y_epuck_current = new_epuck_position[1]

        if (abs(x_box_current - x_epuck_current) < MIN_DISTANCE 
            and abs(y_box_current - y_epuck_current) < MIN_DISTANCE):
            break
        
        if right_front_distance_sensor.getValue() > SENSOR_DISTANCE:
            left_motor.setVelocity(-MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)
            robot.step(800)
        elif left_front_distance_sensor.getValue() > SENSOR_DISTANCE:
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(-MAX_SPEED)
            robot.step(800)
        else:
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)

    for led in leds:
        led.set(1)
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

if __name__ == "__main__":
    main()


