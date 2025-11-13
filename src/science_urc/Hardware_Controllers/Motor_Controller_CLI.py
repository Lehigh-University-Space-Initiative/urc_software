#####################################################################################################################
# Importing Program Libraries
#   - time:
#       - Adds delays to the program
#       - Enables the servo motor to reach a specific position/angle before the PWM signal stops
#   - curses:
#       - Handles live keypresses without pressing "Enter"
#       - Updates the terminal display in real time
#       - Enables the creation of a dynamic CLI (command-line interface)
#   - RPi.GPIO:
#       - Controls the Raspberry Pi GPIO pins
#       - Enables configuration of RPI pins as outputs for DC/servo motors
#       - Enables generating PWM signals to drive motor driver and servo
#####################################################################################################################

import time
import curses
import RPi.GPIO as GPIO




#####################################################################################################################
# Declaring Program Variables
#   - GPIO pins values for all motors and sensor inputs
#   - Default PWM frequencies (in Hz) for different motor types
#       - DC brushed motors: ~100Hz
#       - Servo motors: ~50Hz
#####################################################################################################################

#GPIO Pins
motor1Pin = 24
motor2Pin = 6
servoMotorPin = 27
#sparkSensorPin = 24   # Spark MAX feedback signal pin (input mode)

#PWM Frequencies
pwmFrequency_DC = 100
pwmFrequency_Servo = 50

#Speed and control parameters
DEFAULT_START_SPEED = 10     # Default startup duty % (~1550 µs equivalent)
SPEED_STEP = 5               # Incremental step for adjustments
motor1_active = False
motor2_active = False
motor1_speed = 0
motor2_speed = 0




#####################################################################################################################
# GPIO SETUP
#   - Configure Raspberry Pi GPIO pins as outputs
#   - These GPIO pins send PWM signals to the motor driver
#   - The motor driver then regulates the actual power delivered to the DC/servo motors
#   - The Spark MAX sensor pin is configured as an input for reading encoder or status feedback
#####################################################################################################################

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1Pin, GPIO.OUT)
GPIO.setup(motor2Pin, GPIO.OUT)
GPIO.setup(servoMotorPin, GPIO.OUT)
#GPIO.setup(sparkSensorPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Safe 3.3V input mode




#####################################################################################################################
# Initializing the PWM Objects
#   - Each DC motor is assigned a PWM object with a frequency of 100 Hz
#   - The servo motor is assigned a PWM object with a frequency of 50 Hz (standard for hobby servos)
#   - All PWM objects are started with a 0% duty cycle (motors and servo off by default)
#####################################################################################################################

motor1_pwm = GPIO.PWM(motor1Pin, pwmFrequency_DC)
motor2_pwm = GPIO.PWM(motor2Pin, pwmFrequency_DC)
servo_pwm = GPIO.PWM(servoMotorPin, pwmFrequency_Servo)

motor1_pwm.start(0)
motor2_pwm.start(0)
servo_pwm.start(0)




#####################################################################################################################
# Logging Motor and Sensor Actions
#   - Every DC/servo motor action is logged with a timestamp
#   - The Spark MAX sensor pin state can also be monitored in real time
#####################################################################################################################

def log(msg):
    timestamp = time.strftime("[%H:%M:%S]")
    print(f'{timestamp} {msg}')

def read_spark_sensor():
    """Reads the state of the Spark MAX feedback pin"""
    state = 0  # placeholder (GPIO.input(sparkSensorPin))
    log(f"Spark MAX sensor (GPIO24) state: {state}")
    return state




#####################################################################################################################
# Setting Control to DC Motor Configuration
#   - Parameter(s):
#       - motor: the PWM object for either motor1 or motor2
#       - duty: the PWM duty cycle (0–100%)
#       - name: a string label for logging purposes
#   - Changes the PWM duty cycle of the motor
#   - Logs the action with the duty cycle value
#####################################################################################################################

def set_motor(motor, duty, name):
    motor.ChangeDutyCycle(duty)
    log(f'{name} duty={duty}%')




#####################################################################################################################
# Setting Control to Servo Motor Configuration
#   - Parameter(s):
#       - angle: target angle for the servo (0–180 degrees)
#   - Converts the angle into a PWM duty cycle using the formula (angle / 18 + 2)
#   - Sends the duty cycle to the servo to move to the desired position
#   - Waits briefly to allow servo to reach position before stopping the signal
#   - Stops signal after movement to reduce jitter
#####################################################################################################################

def set_servo(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180

    dutyCycle = angle / 18 + 2
    servo_pwm.ChangeDutyCycle(dutyCycle)
    log(f'Servo motor angle set to {angle}°')
    time.sleep(0.3)
    servo_pwm.ChangeDutyCycle(0)




#####################################################################################################################
# Dynamic CLI for Motor Controlling
#   - Updates the terminal continuously to show current DC/servo motor states
#   - Key mappings:
#       - 1: Start Motor1 (+10% duty)
#       - 2: Start Motor2 (+10% duty)
#       - ↑ / ↓: Increment/Decrement Motor1 by ±5%
#       - → / ←: Increment/Decrement Motor2 by ±5%
#       - a: Servo moves to 0 degrees
#       - d: Servo moves to 90 degrees
#       - f: Servo moves to 180 degrees
#       - s: Stop both motors (0% duty cycle)
#       - q: Quit the program
#####################################################################################################################

def CLI_options():
    lines = [
        "Keybind Controls:",
        "- 1: Start Motor1 (+10% duty)",
        "- 2: Start Motor2 (+10% duty)",
        "- ↑ / ↓ : Motor1 speed ±5 %",
        "- → / ← : Motor2 speed ±5 %",
        "- a / d / f : servo to 0°, 90°, 180°",
        "- s : stop all motors (0% duty)",
        "- q : quit program",
    ]
    return "\n".join(lines)


def CLI(stdscr):
    global motor1_active, motor2_active, motor1_speed, motor2_speed
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    msg = "Motors inactive. Use 1 or 2 to start."

    while True:
        stdscr.clear()
        stdscr.addstr(CLI_options())
        stdscr.addstr("\n----------------------------------------------\n")
        stdscr.addstr(f"Motor1: {'ACTIVE' if motor1_active else 'INACTIVE'} | {motor1_speed}%\n")
        stdscr.addstr(f"Motor2: {'ACTIVE' if motor2_active else 'INACTIVE'} | {motor2_speed}%\n")
        stdscr.addstr("Servo: ready\n")
        stdscr.addstr("Peristaltic Pump #1 : [reserved]\n")
        stdscr.addstr("Peristaltic Pump #2 : [reserved]\n")
        stdscr.addstr(f"Current Action: {msg}")
        stdscr.refresh()
        key = stdscr.getch()

        if key == -1:
            time.sleep(0.1)
            continue

        # Manual motor activation
        if key == ord('1') and not motor1_active:
            motor1_speed = DEFAULT_START_SPEED
            set_motor(motor1_pwm, motor1_speed, "Motor1 Started")
            motor1_active = True
            msg = "Motor1 started at +10%"
        elif key == ord('2') and not motor2_active:
            motor2_speed = DEFAULT_START_SPEED
            set_motor(motor2_pwm, motor2_speed, "Motor2 Started")
            motor2_active = True
            msg = "Motor2 started at +10%"

        # Adjustments (only if active)
        elif key == curses.KEY_UP and motor1_active:
            motor1_speed = min(motor1_speed + SPEED_STEP, 100)
            set_motor(motor1_pwm, motor1_speed, "Motor1 Forward")
            msg = "Motor1 speed increased"
        elif key == curses.KEY_DOWN and motor1_active:
            motor1_speed = max(motor1_speed - SPEED_STEP, 0)
            set_motor(motor1_pwm, motor1_speed, "Motor1 Backward")
            msg = "Motor1 speed decreased"
        elif key == curses.KEY_RIGHT and motor2_active:
            motor2_speed = min(motor2_speed + SPEED_STEP, 100)
            set_motor(motor2_pwm, motor2_speed, "Motor2 Forward")
            msg = "Motor2 speed increased"
        elif key == curses.KEY_LEFT and motor2_active:
            motor2_speed = max(motor2_speed - SPEED_STEP, 0)
            set_motor(motor2_pwm, motor2_speed, "Motor2 Backward")
            msg = "Motor2 speed decreased"

        # Servo keys
        elif key in (ord('a'), ord('d'), ord('f')):
            angle = {"a": 0, "d": 90, "f": 180}[chr(key)]
            set_servo(angle)
            msg = f"Servo {angle}°"

        # Stop all motors
        elif key == ord('s'):
            set_motor(motor1_pwm, 0, "Motor1 Stop")
            set_motor(motor2_pwm, 0, "Motor2 Stop")
            motor1_active = False
            motor2_active = False
            motor1_speed = 0
            motor2_speed = 0
            msg = "Motors stopped and set to 0%"

        # Quit program
        elif key == ord('q'):
            break

        time.sleep(0.1)




#####################################################################################################################
# Main Execution and Cleanup
#   - Runs the CLI in a curses wrapper for safe terminal handling
#   - Ensures all PWM signals are stopped at the end of the program
#   - Cleans up GPIO to release the pins
#   - Explicitly stops PWM before cleanup to prevent lgpio TypeError warnings
#####################################################################################################################

try:
    curses.wrapper(CLI)
finally:
    # Stop all PWM outputs before cleanup
    motor1_pwm.stop()
    motor2_pwm.stop()
    servo_pwm.stop()

    del motor1_pwm
    del motor2_pwm
    del servo_pwm
    GPIO.cleanup()
    print("Motors stopped, GPIO cleaned up safely.")