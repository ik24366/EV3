#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox
from pybricks.tools import wait

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# Establish Bluetooth connection
client = BluetoothMailboxClient()
mbox = TextMailbox('actions', client)

print('Connecting to server...')
client.connect('ev3-server')  # Replace 'ev3-server' with the server EV3's Bluetooth name
print('Connected!')

BASE_SPEED = 170
SLOW_SPEED = BASE_SPEED // 2

def execute_action(action):
    """
    This function parses and executes actions received from the server.
    """
    if action == "follow_line":
        robot.drive(BASE_SPEED, 0)
    elif action == "object_detected":
        robot.stop()
        ev3.speaker.beep()
    elif action == "slow_down":
        robot.drive(SLOW_SPEED, 0)
    elif action == "stop":
        robot.stop()
    elif action.startswith("turn:"):
        _, angle = action.split(":")
        robot.turn(int(angle))
    elif action == "parking_complete":
        ev3.speaker.beep()
        print("Parking complete")
        exit()
    else:
        print(f"Unknown action: {action}")

# Main loop
while True:
    print("Waiting for action from server...")
    mbox.wait()  # Wait for the next message from the server
    action = mbox.read()
    print(f"Received action: {action}")
    execute_action(action)
    wait(100)
