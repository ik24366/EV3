#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxServer, TextMailbox
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
WTR = 28
SLT = 20

w5 = 500
w10 = 1100


# Initialize the server for communication
server = BluetoothMailboxServer()

# Initialize mailboxes for communication
lane_mailbox = TextMailbox('lane', server)
yellow_mailbox = TextMailbox('yellow', server)
blue_mailbox = TextMailbox('blue', server)
# Initialize mailboxes
sync_mailbox = TextMailbox('sync', server)  # New mailbox for synchronization



# Wait for connection from the follower
server.wait_for_connection()
ev3.screen.print("Connected to Follower")



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
OBJECT_DETECTION_THRESHOLD = 230 



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


BDone = {
    "Blue": False
}

YDone = {
    "Yellow": False
}




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
    for _ in range(80):  # Wait for 5 seconds for manual recalibration option
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
    if is_color_match(sensor_rgb, calibrated_rgb, BLUE_WEIGHTS, BLUE_TOLERANCE) and not BDone["Blue"]:
        BDone["Blue"] = True  # Mark blue as handled
        ev3.screen.print("BLUE")
        send_blue_command()
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
    if is_color_match(sensor_rgb, calibrated_rgb, YELLOW_WEIGHTS, YELLOW_TOLERANCE) and not YDone["Yellow"]:
        YDone["Yellow"] = True  # Mark yellow as handled
        ev3.screen.print("YELLOW")
        send_yellow_command()
        robot.drive(speed, 0)
        wait(200)
        robot.drive(speed / 2, 0)  # Slow down
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


# Function to handle lane switching
def handle_lane_switch():
    global lane
    # Check if the LEFT or RIGHT button is pressed
    if Button.LEFT in ev3.buttons.pressed():  # Use LEFT button for lane switching
        lane = 'LEFT'
        ev3.screen.clear()
        ev3.screen.print("Lane switched to LEFT")
    elif Button.RIGHT in ev3.buttons.pressed():  # Use RIGHT button for lane switching
        lane = 'RIGHT'
        ev3.screen.clear()
        ev3.screen.print("Lane switched to RIGHT")
    # Wait until the button is released to prevent multiple toggles
    while Button.LEFT in ev3.buttons.pressed() or Button.RIGHT in ev3.buttons.pressed():
        wait(10)





def switch_lane():
    global lane
    if lane == 'LEFT':

        send_lane_command("RIGHT")

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

        # Turn back 90 degrees to align with the new lane
        robot.turn(-70)
        wait(400)

        # Update the lane
        lane = 'RIGHT'

    elif lane == 'RIGHT':

        send_lane_command("LEFT")

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

        # Turn back 90 degrees to align with the new lane
        robot.turn(70)
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



# Parking behavior
def handle_parking():
    global parking_mode, front_ultrasonic_active
    if parking_mode:
        back_distance = ultrasonic_sensor_back.distance()
        ev3.screen.print("Back distance:", back_distance)
        if back_distance < OBJECT_DETECTION_THRESHOLD_Park_BACK:  
            robot.drive(speed, 0)
            wait(1000)
            robot.turn(65)  # Turn 90 degrees to the right
            front_ultrasonic_active = True  # Activate the front sensor
            ev3.screen.print("Front sensor active")
            robot.drive(speed, 0)
            while True:
                front_distance = ultrasonic_sensor_front.distance()
                ev3.screen.print("Distance:", front_distance)
                if front_distance <= OBJECT_DETECTION_THRESHOLD_Park_FRONT:  
                    ev3.screen.print("Parking Complete")
                    robot.stop()
                    wait(999999)
                    break


# Send commands to followers in relevant scenarios
def send_lane_command(new_lane):
    lane_mailbox.send("lane:" + new_lane)
    ev3.screen.print("Sent Lane Command: " + new_lane)

def send_yellow_command():
    yellow_mailbox.send("zone:YELLOW")
    ev3.screen.print("Sent Yellow Zone Command")

def send_blue_command():
    blue_mailbox.send("stop:BLUE")
    ev3.screen.print("Sent Blue Block Stop Command")

lane_switch_counter = 0  # Counter for lane switch commands

def send_lane_command(new_lane):
    global lane_switch_counter
    lane_switch_counter += 1  # Increment counter for each new command
    lane_mailbox.send("lane:" + new_lane + ":" + str(lane_switch_counter))




# Main loop function
def adjust_movement():
    rgb1 = color_sensor1.rgb()  # Left sensor
    rgb2 = color_sensor2.rgb()  # Right sensor

    handle_green(rgb1, rgb2)  # Pass both sensor readings to handle_green
    
    handle_white(rgb1, True)   # True indicates the left sensor
    handle_white(rgb2, False)  # False indicates the right sensor
    
    detect_object() 

    handle_blue(rgb1, True)    
    handle_blue(rgb2, False)  

    handle_yellow(rgb1, True)  
    handle_yellow(rgb2, False) 
    
    handle_red(rgb1, True)
    handle_red(rgb2, False)


    # Call the lane switch handler to check for button presses
    handle_lane_switch()

    if lane:
        ev3.screen.clear()
        ev3.screen.print(lane)
        if front_ultrasonic_active:
            distance = ultrasonic_sensor_front.distance()  # Front sensor distance
        else:
            distance = ultrasonic_sensor_back.distance()  # Back sensor distance
        ev3.screen.print("Dist:", distance)  # Display the distance dynamically

    if parking_mode:
        handle_parking()
    else:
        robot.drive(speed, 0)


    robot.drive(speed, 0)  # Keep driving forward



ev3.speaker.beep()  # Beep once when the program starts

# Perform setup for calibration (replace calibrate_colors call)
setup_calibration()


# Notify follower to start calibration
ev3.screen.print("Notifying Follower...")
sync_mailbox.send("calibrate")

# Wait for follower to confirm readiness
ev3.screen.print("Waiting for Follower Ready...")
while True:
    follower_response = sync_mailbox.wait_new()
    if follower_response == "follower_ready":
        ev3.screen.print(" Ready!")
        break

# Send the "start" signal
sync_mailbox.send("start")
ev3.screen.print("Start")

# Main loop
while True:
    adjust_movement()

    