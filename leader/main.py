#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxServer, TextMailbox

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
left_sensor = ColorSensor(Port.S2)
right_sensor = ColorSensor(Port.S3)
front_ultrasonic_sensor = UltrasonicSensor(Port.S4)
side_ultrasonic_sensor = UltrasonicSensor(Port.S1)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# Bluetooth server setup
server = BluetoothMailboxServer()
mbox = TextMailbox('actions', server)

print('Waiting for client connection...')
server.wait_for_connection()
print('Client connected!')

BASE_SPEED = 170
AVOID_SPEED = BASE_SPEED * 0.6
SLOW_SPEED = BASE_SPEED // 2
TURN_RATE = 45
OBJECT_DETECTION_THRESHOLD = 330 
BACK_ULTRASONIC_THRESHOLD = 175 
FRONT_ULTRASONIC_THRESHOLD = 120  
STOP_PARKING_THRESHOLD = 50 

lane_state = 'left'
red_detection_count = 0  # Added back this global variable
parking_mode = False

def is_color(rgb_value, target_value):
    TOLERANCE = 8
    return all(abs(rgb_value[i] - target_value[i]) <= TOLERANCE for i in range(3))

def display_message(msg):
    ev3.screen.clear()
    ev3.screen.draw_text(0, 0, msg)

def send_status_to_client(message):
    """Send a status message to the connected client."""
    mbox.send(message)
    print("Sent to client: " + message)

def calibrate_color(color_name):
    display_message(color_name + " please")
    while not ev3.buttons.pressed():
        wait(10)

    left_rgb = left_sensor.rgb()
    right_rgb = right_sensor.rgb()

    display_message(color_name + " calibrated")
    print(color_name, "left sensor RGB:", left_rgb)
    print(color_name, "right sensor RGB:", right_rgb)

    return left_rgb, right_rgb

def calibrate_colors():
    display_message("Starting color calibration")

    green_left, green_right = calibrate_color("green")
    wait(1000)
    white_left, white_right = calibrate_color("white")
    wait(1000)
    blue_left, blue_right = calibrate_color("blue")
    wait(1000)
    yellow_left, yellow_right = calibrate_color("yellow")
    wait(1000)
    red_left, red_right = calibrate_color("red")

    display_message("Calibration complete!")
    wait(1000)
    robot.drive_time(BASE_SPEED, 0, 1000)

    return {
        'GREEN_LEFT': green_left, 'GREEN_RIGHT': green_right,
        'WHITE_LEFT': white_left, 'WHITE_RIGHT': white_right,
        'BLUE_LEFT': blue_left, 'BLUE_RIGHT': blue_right,
        'YELLOW_LEFT': yellow_left, 'YELLOW_RIGHT': yellow_right,
        'RED_LEFT': red_left, 'RED_RIGHT': red_right
    }

color_values = calibrate_colors()

BUFFER_ZONE = 5

def follow_line(speed=BASE_SPEED):
    left_reflection = left_sensor.reflection()
    right_reflection = right_sensor.reflection()

    deviation = left_reflection - right_reflection
    gain = 2.6  

    if abs(deviation) < BUFFER_ZONE:
        deviation = 0

    robot.drive(speed, deviation * gain)

    if lane_state == 'left' and is_color(left_sensor.rgb(), color_values['GREEN_LEFT']):
        robot.drive(speed, deviation * (gain * 0.4))
    elif lane_state == 'right' and is_color(right_sensor.rgb(), color_values['GREEN_RIGHT']):
        robot.drive(speed, deviation * (gain * 0.4))

def detect_lane():
    global lane_state

    left_rgb = left_sensor.rgb()
    right_rgb = right_sensor.rgb()

    if is_color(left_rgb, color_values['GREEN_LEFT']):
        lane_state = 'left'
        send_status_to_client("Lane: Left")
    elif is_color(right_rgb, color_values['GREEN_RIGHT']):
        lane_state = 'right'
        send_status_to_client("Lane: Right")

def detect_object():
    global lane_state
    distance = front_ultrasonic_sensor.distance()

    if not parking_mode and distance < OBJECT_DETECTION_THRESHOLD:
        send_status_to_client("Object detected")
        ev3.speaker.beep()

        if lane_state == 'left':
            while front_ultrasonic_sensor.distance() < OBJECT_DETECTION_THRESHOLD:
                robot.drive(AVOID_SPEED, TURN_RATE)
            lane_state = 'right'
            send_status_to_client("Switched to right lane")
            adjust_lane_after_avoid("right")
        elif lane_state == 'right':
            while front_ultrasonic_sensor.distance() < OBJECT_DETECTION_THRESHOLD:
                robot.drive(AVOID_SPEED, -TURN_RATE)
            lane_state = 'left'
            send_status_to_client("Switched to left lane")
            adjust_lane_after_avoid("left")

def adjust_lane_after_avoid(target_lane, max_distance=240):
    global lane_state
    robot.reset()

    while True:
        current_distance = robot.distance()
        if abs(current_distance) > max_distance:
            send_status_to_client("Adjustment limit reached")
            break

        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        deviation = left_reflection - right_reflection

        if target_lane == "right":
            if not is_color(right_sensor.rgb(), color_values['GREEN_RIGHT']):
                robot.drive(BASE_SPEED, -20) 
            else:
                send_status_to_client("Aligned to right lane")
                break
        elif target_lane == "left":
            if not is_color(left_sensor.rgb(), color_values['GREEN_LEFT']):
                robot.drive(BASE_SPEED, 20)
            else:
                send_status_to_client("Aligned to left lane")
                break
        wait(50)

    robot.drive(BASE_SPEED, 0)

def handle_color():
    global red_detection_count, parking_mode

    left_rgb = left_sensor.rgb()
    right_rgb = right_sensor.rgb()

    if is_color(left_rgb, color_values['BLUE_LEFT']) or is_color(right_rgb, color_values['BLUE_RIGHT']):
        send_status_to_client("Blue detected")
        robot.stop()
        ev3.speaker.beep()
        wait(3000)

    elif is_color(left_rgb, color_values['YELLOW_LEFT']) or is_color(right_rgb, color_values['YELLOW_RIGHT']):
        send_status_to_client("Yellow detected - slowing down")
        for _ in range(20):  
            follow_line(speed=SLOW_SPEED)
            wait(100)

    elif is_color(left_rgb, color_values['RED_LEFT']) or is_color(right_rgb, color_values['RED_RIGHT']):
        send_status_to_client("Red detected")
        red_detection_count += 1
        if red_detection_count == 2:
            parking_mode = True

def park_after_red():
    if parking_mode:
        send_status_to_client("Parking mode activated")
        while side_ultrasonic_sensor.distance() > BACK_ULTRASONIC_THRESHOLD:
            follow_line()
        robot.drive_time(SLOW_SPEED, 0, 2000)
        robot.turn(90)

        while front_ultrasonic_sensor.distance() > STOP_PARKING_THRESHOLD:
            robot.drive(SLOW_SPEED, 0)

        robot.stop()
        send_status_to_client("Parking complete")
        ev3.speaker.beep()
        exit()

while True:
    if not parking_mode:
        detect_object()
        follow_line()
        handle_color()
        detect_lane()
    else:
        park_after_red()
    wait(10)
