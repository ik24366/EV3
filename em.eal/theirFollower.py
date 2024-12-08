#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox
import os
import json


# Initialize EV3
ev3 = EV3Brick()
motor_left = Motor(Port.A)
motor_right = Motor(Port.B)

# drivebase
WHEEL_DIAMETER = 50  # in millimeters (adjust to your robot)
AXLE_TRACK = 160     # distance between the centers of the two wheels (adjust to your robot)

robot = DriveBase(motor_left, motor_right, WHEEL_DIAMETER, AXLE_TRACK)

# Sensors
color_sensor1 = ColorSensor(Port.S1)  # Left sensor
color_sensor2 = ColorSensor(Port.S2)  # Right sensor
ultrasonic_sensor_front = UltrasonicSensor(Port.S4)  # Front sensor
ultrasonic_sensor_back = UltrasonicSensor(Port.S3)  # Back sensor

# speed
speed = 90  # mm/s
slspeed = 90
WTR = 32
SLT = 20

w5 = 500
w10 = 1000


# Initialize the client and mailbox
client = BluetoothMailboxClient()

# Initialize mailboxes for communication
lane_mailbox = TextMailbox('lane', client)
yellow_mailbox = TextMailbox('yellow', client)
blue_mailbox = TextMailbox('blue', client)
# Initialize mailboxes
sync_mailbox = TextMailbox('sync', client)  # New mailbox for synchronization
parking_mailbox = TextMailbox('parking', client)  # New mailbox for parking mode



start_parking_mailbox = TextMailbox('start_parking', client)
turn_mailbox = TextMailbox('turn', client)
reverse_mailbox = TextMailbox('reverse', client)
parked_mailbox = TextMailbox('parked', client)


# Connect to the leader EV3
client.connect('ev3-momo')  # Replace 'ev3-leader' with the leader's Bluetooth name or address
ev3.screen.print("Connected to Leader")


# Initialize dictionaries for tracking command handling
BDone = {
    "Blue": False
}

YDone = {
    "Yellow": False
}

lane_switching = False  # Track if lane switching is in progress


# turn rate
turn_rate = 60  # degrees per second
slturn_rate = 75


# Define individual tolerance levels for each color
WHITE_TOLERANCE = 20
GREEN_TOLERANCE = 15
BLUE_TOLERANCE = 6
YELLOW_TOLERANCE = 12
RED_TOLERANCE = 10


# Define individual weights for each color
WHITE_WEIGHTS = (2.0, 0.5, 0.5)
GREEN_WEIGHTS = (0.5, 0.5, 2.0)
BLUE_WEIGHTS = (0.5, 0.5, 1.8)
YELLOW_WEIGHTS = (2.0, 2.0, 0.5)
RED_WEIGHTS = (2.0, 0.5, 0.5)

lane = None  # Lane can be 'LEFT' or 'RIGHT'


# Object detection distance 
OBJECT_DETECTION_THRESHOLD = 290  



# Object detection thresholds
OBJECT_DETECTION_THRESHOLD_Park_FRONT = 100  # mm
OBJECT_DETECTION_THRESHOLD_Park_BACK = 150 # mm


# Stopwatch for object detection delay
object_stopwatch = StopWatch()
OBJECT_DETECTION_DELAY = 2700  # Delay in milliseconds

# Stopwatch for red detection delay
red_stopwatch = StopWatch()
RED_DETECTION_DELAY = 2000  # 5 seconds delay after yellow


# Add this global variable to keep track of the last turn direction
last_turn_direction = 0  # 0 for straight, positive for right, negative for left

parking_mode = False  # Flag for parking mode
front_ultrasonic_active = True  # Front ultrasonic sensor active by default


# Calibrated colors dictionary
calibrated_colors = {
    'black': {'sensor1': None, 'sensor2': None},
    'white': {'sensor1': None, 'sensor2': None},
    'green': {'sensor1': None, 'sensor2': None},
    'blue': {'sensor1': None, 'sensor2': None},
    'yellow': {'sensor1': None, 'sensor2': None},
    'red': {'sensor1': None, 'sensor2': None}
}

# File to save calibration data
CALIBRATION_FILE = "calibration_data.json"

# Save calibration data to a file
def save_calibration_data():
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(calibrated_colors, f)
    ev3.screen.print("Calibration Saved")

# Load calibration data from a file
def load_calibration_data():
    global calibrated_colors
    try:
        with open(CALIBRATION_FILE, "r") as f:
            calibrated_colors = json.load(f)
        ev3.screen.print("Calibration Loaded")
        return True
    except (OSError, IOError, json.JSONDecodeError):
        ev3.screen.print("No Calibration Found")
        return False


# Calibration function
def calibrate_colors():
    colors = ['black', 'white', 'green', 'blue', 'yellow', 'red']
    num_samples = 8  # Number of samples to take for each color

    for color in colors:
        ev3.screen.clear()
        ev3.screen.print("Calibrate {}".format(color.upper()))
        ev3.screen.print("Place sensors over")
        ev3.screen.print("{} and press".format(color.upper()))
        ev3.screen.print("CENTER button")

        # Wait for the user to press the center button
        while Button.CENTER not in ev3.buttons.pressed():
            wait(10)
        # Wait until the button is released
        while any(ev3.buttons.pressed()):
            wait(10)

        # Collect samples for each sensor
        samples1 = []
        samples2 = []
        for i in range(num_samples):
            rgb1 = color_sensor1.rgb()
            rgb2 = color_sensor2.rgb()
            if rgb1 is not None:
                samples1.append(rgb1)
            if rgb2 is not None:
                samples2.append(rgb2)
            wait(100)  # Small delay between samples

        # Compute average RGB for each sensor
        avg_rgb1 = tuple(sum(vals) / len(vals) for vals in zip(*samples1))
        avg_rgb2 = tuple(sum(vals) / len(vals) for vals in zip(*samples2))

        # Store the average values in the calibrated_colors dictionary
        calibrated_colors[color]['sensor1'] = avg_rgb1
        calibrated_colors[color]['sensor2'] = avg_rgb2

    ev3.screen.clear()
    ev3.screen.print("Calibration Completed")
    wait(2000)

def setup_calibration():
    ev3.screen.print("Up for Recalibration")
    for _ in range(50):  # Wait for 5 seconds for manual recalibration option
        if Button.UP in ev3.buttons.pressed():  # Use UP button for calibration
            calibrate_colors()
            save_calibration_data()
            return
        wait(100)
    # If no recalibration, try to load existing data
    if not load_calibration_data():
        ev3.screen.print("No Calibration Found")
        calibrate_colors()
        save_calibration_data()


# Check if the detected color matches the calibrated value
def is_color_match(sensor_rgb, calibrated_rgb, weights, tolerance):
    if sensor_rgb is None or calibrated_rgb is None:
        return False
    weighted_difference = sum(
        weights[i] * abs(sensor_rgb[i] - calibrated_rgb[i]) for i in range(3)
    )
    return weighted_difference <= tolerance



def handle_white(sensor_rgb, is_left_sensor):
    global lane
    sensor = 'sensor1' if is_left_sensor else 'sensor2'
    calibrated_rgb = calibrated_colors['white'][sensor]

    if is_color_match(sensor_rgb, calibrated_rgb, WHITE_WEIGHTS, WHITE_TOLERANCE):
        if is_left_sensor:  # Logic for the left sensor
            if lane == 'RIGHT':  # If on the RIGHT lane
                robot.drive(speed, turn_rate)  # Normal right turn
                wait(25)
            elif lane == 'LEFT':  # If on the LEFT lane
                robot.drive(speed, -(turn_rate + WTR))  # Sharp left turn
                wait(50)
        else:  # Logic for the right sensor
            if lane == 'RIGHT':  # If on the RIGHT lane
                robot.drive(speed, (turn_rate + WTR))  # Sharp right turn
                wait(50)
            elif lane == 'LEFT':  # If on the LEFT lane
                robot.drive(speed, -turn_rate)  # Normal left turn
                wait(25)



# Handle green color detection
def handle_green(sensor1_rgb, sensor2_rgb):
    global lane, last_turn_direction
    # Check if each sensor detects green
    sensor1_green = is_color_match(sensor1_rgb, calibrated_colors['green']['sensor1'], GREEN_WEIGHTS, GREEN_TOLERANCE)
    sensor2_green = is_color_match(sensor2_rgb, calibrated_colors['green']['sensor2'], GREEN_WEIGHTS, GREEN_TOLERANCE)

    if sensor1_green and sensor2_green:
        robot.stop()  # Stop the robot
        wait(500)  # Pause for half a second to ensure a complete stop

        if last_turn_direction >= 0:
            robot.turn(40)  # Turn 40 degrees to the right
            ev3.screen.print("G BOTH - RIGHT")
            last_turn_direction = 1
            robot.drive(speed, 0)

        else:
            robot.turn(-40)  # Turn 40 degrees to the left
            ev3.screen.print("G BOTH - LEFT")
            last_turn_direction = -1
            robot.drive(speed, 0)

    elif sensor1_green:
        if lane is None:
            lane = 'LEFT'
        robot.drive(speed, turn_rate)  # Turn left
        last_turn_direction = -1
        ev3.screen.print("GREEN LEFT")


    elif sensor2_green:
        if lane is None:
            lane = 'RIGHT'
        robot.drive(speed, -turn_rate)  # Turn right
        last_turn_direction = 1
        ev3.screen.print("GREEN RIGHT")


# Handle blue color detection
def handle_blue(sensor_rgb, is_left_sensor):
    sensor = 'sensor1' if is_left_sensor else 'sensor2'
    calibrated_rgb = calibrated_colors['blue'][sensor]
    if is_color_match(sensor_rgb, calibrated_rgb, BLUE_WEIGHTS, BLUE_TOLERANCE):
        ev3.screen.print("BLUE")
        robot.drive(speed, 0)
        wait(100)
        robot.stop()
        wait(3000)
        robot.drive(speed, 0)  # Resume driving forward

# Handle yellow color detection
def handle_yellow(sensor_rgb, is_left_sensor):
    global red_stopwatch
    sensor = 'sensor1' if is_left_sensor else 'sensor2'
    calibrated_rgb = calibrated_colors['yellow'][sensor]
    if is_color_match(sensor_rgb, calibrated_rgb, YELLOW_WEIGHTS, YELLOW_TOLERANCE):
        robot.drive(speed, 0)
        wait(200)
        robot.drive(speed / 2, 0)  # Slow down
        ev3.screen.print("YELLOW")
        wait(2300)
        red_stopwatch.reset()
        robot.drive(speed, 0)  # Resume normal speed


# Handle red detection
def handle_red(sensor_rgb, is_left_sensor):
    global red_stopwatch
    sensor = 'sensor1' if is_left_sensor else 'sensor2'
    calibrated_rgb = calibrated_colors['red'][sensor]
    
    if is_color_match(sensor_rgb, calibrated_rgb, RED_WEIGHTS, RED_TOLERANCE):
        if red_stopwatch.time() > RED_DETECTION_DELAY and not parking_mode:
            ev3.screen.print("Parking...")
            enter_parking_mode()  # Enter parking mode

            
            red_stopwatch.reset()  # Reset stopwatch after detecting red


def handle_lane_switch():
    global lane
    # Check if the CENTER button is pressed
    if Button.CENTER in ev3.buttons.pressed():
        # Toggle the lane
        if lane == 'LEFT':
            lane = 'RIGHT'
        else:
            lane = 'LEFT'

        # Provide feedback on the EV3 screen
        ev3.screen.clear()
        ev3.screen.print("Lane switched!")
        ev3.screen.print("Current lane: {}".format(lane))

        # Wait until the button is released to prevent multiple toggles
        while Button.CENTER in ev3.buttons.pressed():
            wait(10)

def switch_lane():
    global lane
    if lane == 'LEFT':
        # Stop the robot before turning
        robot.stop()
        wait(200)

        # Turn 90 degrees to the right
        robot.turn(70)  # Adjust this if 90 degrees needs calibration
        wait(200)

        # Move forward for 200ms
        robot.drive(slspeed, 0)
        wait(w10)
        robot.stop()
        wait(300)

        # Turn back 90 degrees to align with the new lane
        robot.turn(-60)
        wait(400)

        # Update the lane
        lane = 'RIGHT'

    elif lane == 'RIGHT':
        # Stop the robot before turning
        robot.stop()
        wait(200)

        # Turn 90 degrees to the left
        robot.turn(-70)  # Adjust this if 90 degrees needs calibration
        wait(200)

        # Move forward for 200ms
        robot.drive(slspeed, 0)
        wait(w10)
        robot.stop()
        wait(300)

        # Turn back 90 degrees to align with the new lane
        robot.turn(60)
        wait(400)

        # Update the lane
        lane = 'LEFT'



# Function to detect objects based on active ultrasonic sensor
def detect_object():
    if object_stopwatch.time() > OBJECT_DETECTION_DELAY:  # Check if delay has passed
        if front_ultrasonic_active:
            distance = ultrasonic_sensor_front.distance()
        else:
            distance = ultrasonic_sensor_back.distance()
        
        ev3.screen.print("Dist:", distance)  # Display distance dynamically
        if distance < OBJECT_DETECTION_THRESHOLD:
            ev3.screen.print("Object detected!")
            if not parking_mode:
                switch_lane()  # Switch lanes only if not in parking mode
            object_stopwatch.reset()  # Reset stopwatch



# Parking mode setup
def enter_parking_mode():
    global parking_mode, lane, front_ultrasonic_active
    parking_mode = True
    #front_ultrasonic_active = False  # Use the back sensor in parking mode
    if lane == 'LEFT':  # Ensure the robot is in the right lane
        switch_lane()
    lane = 'RIGHT'
    ev3.screen.print("Parking Mode ON")




# Parking behavior for the follower
def handle_parking():
    global parking_mode
    if parking_mode:
        # Wait for the start command
        if start_parking_mailbox.read() == "start":
            ev3.screen.print("Follower: Starting parking")
            robot.drive(speed, 0)
            wait(1000)

        # Wait for the turn command
        if turn_mailbox.read() == "turn":
            ev3.screen.print("Follower: Turning")
            robot.turn(-65)  # Turn 90 degrees to the right

        # Wait for the reverse command
        if reverse_mailbox.read() == "reverse":
            ev3.screen.print("Follower: Reversing")
            robot.drive(-speed, 0)
            wait(2000)

        # Wait for the parked confirmation
        if parked_mailbox.read() == "parked":
            ev3.screen.print("Follower: Parking complete")
            ev3.speaker.beep()
            robot.stop()
            wait(999999)



    
def maintain_distance():
    global speed, lane_switching
    if lane_switching:
        return  # Skip if lane switching is in progress
     
    # Measure distance to the leader using the front ultrasonic sensor
    distance_to_leader = ultrasonic_sensor_front.distance()

    # Desired distance range
    stop = 60
    min_distance = 80  # Minimum safe distance (in mm)
    max_distance = 200  # Maximum safe distance (in mm)

    # Adjust speed dynamically based on the distance
    if distance_to_leader < min_distance:
        # Too close, slow down or stop
        speed = max(70, speed - 10)  # Decrease speed gradually

    elif distance_to_leader > max_distance:
        # Too far, speed up
        speed = min(92, speed + 10)  # Increase speed gradually

    elif distance_to_leader < stop:
        robot.stop()
        wait(500)
        

    # Set robot speed accordingly
    robot.drive(speed, 0)


lane_counter = 0  # Tracks the last processed lane switch command

def process_lane_command(command):
    global lane, lane_counter
    try:
        # Parse the command: "lane:<LEFT/RIGHT>:<counter>"
        parts = command.split(":")
        if len(parts) == 3:
            new_lane = parts[1]
            command_counter = int(parts[2])

            # Check if the command counter is greater than the current lane_counter
            if command_counter > lane_counter:
                lane_counter = command_counter  # Update the counter
                if new_lane != lane:
                    switch_lane()  # Execute lane switch logic
                    lane = new_lane  # Update the current lane
    except Exception as e:
        ev3.screen.print("Error processing lane command: " + str(e))


# Main loop function
def adjust_movement():
    rgb1 = color_sensor1.rgb()  # Left sensor
    rgb2 = color_sensor2.rgb()  # Right sensor

    handle_green(rgb1, rgb2)  # Pass both sensor readings to handle_green
    
    handle_white(rgb1, True)   # True indicates the left sensor
    handle_white(rgb2, False)  # False indicates the right sensor
    
    #detect_object() 

    #handle_blue(rgb1, True)    
    #handle_blue(rgb2, False)  

    #handle_yellow(rgb1, True)  
    #handle_yellow(rgb2, False) 
    
    #handle_red(rgb1, True)
    #handle_red(rgb2, False)


    # Call the lane switch handler to check for button presses
    #handle_lane_switch()

    if lane:
        ev3.screen.clear()
        ev3.screen.print(lane)
        if front_ultrasonic_active:
            distance = ultrasonic_sensor_front.distance()  # Front sensor distance
        else:
            distance = ultrasonic_sensor_back.distance()  # Back sensor distance
        ev3.screen.print("Dist:", distance)  # Display the distance dynamically

    robot.drive(speed, 0)  # Keep driving forward


# Perform calibration
ev3.speaker.beep()  # Beep once when the program starts


# Wait for calibration signal
ev3.screen.print("Waiting for Calibration Signal...")
while True:
    command = sync_mailbox.wait_new()
    if command == "calibrate":
        ev3.screen.print("Calibration Signal Received.")
        break


# Perform setup for calibration (replace calibrate_colors call)
setup_calibration()

# Notify leader of readiness
sync_mailbox.send("follower_ready")
ev3.screen.print("Signal Sent")

# Wait for start signal
ev3.screen.print("Waiting Start")
while True:
    command = sync_mailbox.wait_new()
    if command == "start":
        ev3.screen.print("Start Signal Received. Beginning Operation.")
        break

# Main loop
while True:
    # Continuously adjust movement
    adjust_movement()

    maintain_distance()
    
    try:
        # Read lane mailbox for new commands
        lane_command = lane_mailbox.read()
        if lane_command:
            process_lane_command(lane_command)


        yellow_command = yellow_mailbox.read()
        if yellow_command and not YDone["Yellow"]:  # Check if the yellow command has not been processed
            ev3.screen.print("Yellow Zone Command Received")
            robot.drive(speed, 0)
            wait(200)
            robot.drive(speed / 2, 0)  # Slow down
            wait(2300)
            red_stopwatch.reset()
            robot.drive(speed, 0)  # Resume normal speed
            YDone["Yellow"] = True  # Mark as handled

        # Blue mailbox: Process only once
        blue_command = blue_mailbox.read()
        if blue_command and not BDone["Blue"]:  # Check if the blue command has not been processed
            ev3.screen.print("Blue Block Stop Detected")
            robot.drive(speed, 0)
            wait(100)
            robot.stop()
            wait(3000)
            robot.drive(speed, 0)  # Resume driving forward
            BDone["Blue"] = True  # Mark as handled

        parking_command = parking_mailbox.read()
        if parking_command == "parking_mode_on":
            ev3.screen.print("parking mode!")
            # Add follower-specific parking logic here
            handle_parking()

    except Exception as e:
        ev3.screen.print("Error: " + str(e))

