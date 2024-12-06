#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S2)
front_ultrasonic_sensor = UltrasonicSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# Bluetooth client setup
client = BluetoothMailboxClient()
mbox = TextMailbox('actions', client)

print('Connecting to server...')
client.connect('ev3-euiyeon')  # Replace 'ev3-server' with the Bluetooth name of the leader
print('Connected to server!')

BASE_SPEED = 170
FOLLOW_DISTANCE = 300  # Desired distance from the leader
BUFFER_ZONE = 5

lane_state = 'left'

def is_color(rgb_value, target_value):
    TOLERANCE = 8
    return all(abs(rgb_value[i] - target_value[i]) <= TOLERANCE for i in range(3))

def follow_line(speed=BASE_SPEED):
    """Follow the line while maintaining lane state."""
    left_reflection = left_sensor.reflection()
    right_reflection = right_sensor.reflection()

    deviation = left_reflection - right_reflection
    gain = 2.6  # Proportional gain for turning adjustment

    if abs(deviation) < BUFFER_ZONE:
        deviation = 0

    robot.drive(speed, deviation * gain)

def maintain_distance():
    """Adjust speed to maintain a specific distance from the leader."""
    distance = front_ultrasonic_sensor.distance()

    if distance > FOLLOW_DISTANCE + 50:
        robot.drive(BASE_SPEED + 50, 0)  # Speed up slightly
    elif distance < FOLLOW_DISTANCE - 50:
        robot.drive(BASE_SPEED - 50, 0)  # Slow down slightly
    else:
        robot.drive(BASE_SPEED, 0)  # Maintain current speed

def handle_leader_action(action):
    """Handle actions received from the leader."""
    if action == "Blue detected":
        robot.stop()
        ev3.speaker.beep()
        wait(3000)

    elif action == "Yellow detected - slowing down":
        for _ in range(20):  # Slow down temporarily
            follow_line(speed=BASE_SPEED // 2)
            wait(100)

    elif action == "Red detected":
        ev3.speaker.beep()
        wait(2000)  # Wait to simulate lane-switching or stopping

    elif action == "Lane: Left":
        global lane_state
        lane_state = 'left'

    elif action == "Lane: Right":
        global lane_state
        lane_state = 'right'

def main():
    """Main loop to follow the leader."""
    while True:
        if mbox.wait_new():  # Wait for a new message from the leader
            action = mbox.read()
            print("Received from leader:", action)
            handle_leader_action(action)

        # Perform lane following and maintain distance from the leader
        follow_line()
        maintain_distance()
        wait(10)

# Start the follower logic
main()
