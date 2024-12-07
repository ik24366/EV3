#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox

# Initialize EV3
ev3 = EV3Brick()
motor_left = Motor(Port.B)
motor_right = Motor(Port.C)

# drivebase
WHEEL_DIAMETER = 50  # in millimeters (adjust to your robot)
AXLE_TRACK = 160     # distance between the centers of the two wheels (adjust to your robot)

robot = DriveBase(motor_left, motor_right, WHEEL_DIAMETER, AXLE_TRACK)

# Sensors
color_sensor1 = ColorSensor(Port.S2)  # Left sensor
color_sensor2 = ColorSensor(Port.S3)  # Right sensor
ultrasonic_sensor_front = UltrasonicSensor(Port.S4)  # Front sensor
ultrasonic_sensor_back = UltrasonicSensor(Port.S1)  # Back sensor


# speed
speed = 110  # mm/s
slspeed = 120
WTR = 28
SLT = 20


# Initialize the client and mailbox
client = BluetoothMailboxClient()
command_mailbox = TextMailbox('command', client)

# Connect to the leader EV3
client.connect('ev3-momo')  # Replace 'ev3-leader' with the leader's Bluetooth name or address
ev3.screen.print("Connected to Leader")


# turn rate
turn_rate = 60  # degrees per second


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
    ev3.screen.print("Calibration")
    ev3.screen.print("Completed")
    wait(2000)

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
        # Smooth turn to the right
        robot.drive(slspeed, (turn_rate - SLT))  # Slow down slightly for smoother turn
        wait(500)# Adjust timing based on track
        lane = 'RIGHT'  # Update to RIGHT lane
        robot.drive(slspeed, 0)
        wait(1000)
        robot.drive(slspeed, -(turn_rate - SLT))
        wait(500)

    elif lane == 'RIGHT':
        # Smooth turn to the left
        robot.drive(slspeed, -(turn_rate - SLT))  # Slow down slightly for smoother turn
        wait(500) # Adjust timing based on track
        lane = 'LEFT'  # Update to LEFT lane
        robot.drive(slspeed, 0)  # Resume driving straight
        wait(1000)
        robot.drive(slspeed, (turn_rate - SLT))
        wait(500)



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

# Wait for the leader's calibration complete signal
while True:
    leader_signal = command_mailbox.wait_new()
    if leader_signal == "calibration_complete":
        ev3.screen.print("Leader calibration complete!")
        break

# Perform calibration
ev3.speaker.beep()  # Beep once when the program starts
calibrate_colors()

# Notify the leader that the follower is ready
command_mailbox.send("follower_ready")
ev3.screen.print("Follower is ready. Waiting for leader to start...")

# Main loop
while True:
    command = command_mailbox.wait_new()  # Wait for a new command

    # Parse and act on the command
    if "lane" in command:
        new_lane = command.split(":")[1]
        ev3.screen.print("Change Lane to:", new_lane)
        if new_lane == 'RIGHT':
            robot.drive(slspeed, turn_rate)
            wait(500)
        elif new_lane == 'LEFT':
            robot.drive(slspeed, -turn_rate)
            wait(500)

    elif "zone:YELLOW" in command:
        ev3.screen.print("Entering Yellow Zone")
        robot.drive(speed / 2, 0)  # Slow down to half speed

    elif "stop:BLUE" in command:
        ev3.screen.print("Blue Block Stop")
        robot.stop()
        wait(3000)  # Stop for 3 seconds
        robot.drive(speed, 0)  # Resume driving forward

    # Continue the follower's base movement logic
    adjust_movement()


