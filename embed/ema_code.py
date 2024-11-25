#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox
from pybricks.tools import wait

# Initialize EV3 and motors
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# Ultrasonic sensor for maintaining distance
front_ultrasonic_sensor = UltrasonicSensor(Port.S4)

# Bluetooth setup
client = BluetoothMailboxClient()
mbox = TextMailbox('command', client)

print("Connecting to leader...")
client.connect('ev3-euiyeon')  # Replace 'ev3-euiyeon' with the Bluetooth name of the leader EV3
print("Connected to leader!")

# Constants for distance-based following
FOLLOW_DISTANCE = 200  # Target distance from the leader (in mm)
FOLLOW_TOLERANCE = 50  # Acceptable range around the target distance
SLOW_DOWN_DISTANCE = FOLLOW_DISTANCE // 2  # Slow down if closer than this

# Main loop to receive and execute commands
while True:
    # Measure the distance to the leader
    current_distance = front_ultrasonic_sensor.distance()

    # Adjust speed based on the distance to the leader
    if current_distance < SLOW_DOWN_DISTANCE:
        # Too close: Stop or reverse slightly
        robot.stop()
        print("Too close to leader. Stopping.")
    elif current_distance > FOLLOW_DISTANCE + FOLLOW_TOLERANCE:
        # Too far: Increase speed
        robot.drive(200, 0)
        print("Too far from leader. Speeding up.")
    elif FOLLOW_DISTANCE - FOLLOW_TOLERANCE <= current_distance <= FOLLOW_DISTANCE + FOLLOW_TOLERANCE:
        # Within the target distance range: Follow the leader's commands
        command = mbox.read()  # Receive a command from the leader
        
        if command is not None:  # Check if a command was received
            print(f"Received command: {command}")  # Debugging output
            if command.startswith("FORWARD:"):
                # Extract speed and deviation values from the command
                params = command.split(":")[1].split(",")
                speed = int(params[0])
                deviation = float(params[1])
                robot.drive(speed, deviation)

            elif command == "STOP":
                robot.stop()

            elif command == "OBJECT_DETECTED":
                ev3.speaker.beep()  # Optional: Signal object detection on follower
                robot.stop()  # Stop until new instructions are received

            elif command.startswith("AVOID_LEFT"):
                # Extract speed and turn rate from the command
                params = command.split(":")[1].split(",")
                speed = int(params[0])
                turn_rate = float(params[1])
                robot.drive(speed, turn_rate)

            elif command.startswith("AVOID_RIGHT"):
                # Extract speed and turn rate from the command
                params = command.split(":")[1].split(",")
                speed = int(params[0])
                turn_rate = float(params[1])
                robot.drive(speed, turn_rate)

            elif command.startswith("ADJUST:right"):
                # Extract speed and turn rate from the command
                params = command.split(":")[2].split(",")
                speed = int(params[0])
                turn_rate = float(params[1])
                robot.drive(speed, turn_rate)

            elif command.startswith("ADJUST:left"):
                # Extract speed and turn rate from the command
                params = command.split(":")[2].split(",")
                speed = int(params[0])
                turn_rate = float(params[1])
                robot.drive(speed, turn_rate)

            elif command == "SLOW_DOWN":
                robot.drive(robot.speed() // 2, 0)  # Reduce the speed by half

            elif command == "LANE:LEFT":
                print("Following leader in the left lane")
                # Optionally add lane-specific behaviors if needed

            elif command == "LANE:RIGHT":
                print("Following leader in the right lane")
                # Optionally add lane-specific behaviors if needed

            elif command == "PARKING_MODE":
                print("Entering parking mode")
                # Optionally handle any additional parking actions
        else:
            print("No command received.")
    
    wait(10)
