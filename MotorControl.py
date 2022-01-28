import RPi.GPIO as GPIO
import sys
import time

# PWM0 and PWM1 for enable pins these are pins 32 33 respectably
# Use 7 8 10 and 11 pins for digital I/O
# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)
mode = GPIO.getmode()
print(" mode ="+str(mode))
GPIO.cleanup()

# Define GPIO signals to use
# Physical pins 11,15,16,18
# GPIO17,GPIO22,GPIO23,GPIO24
#NONE OF THESE PINS ARE RIGHT CURRENTLY DO NOT TRUST
RightPWM = 15
ForwardRight = 16
BackwardRight = 17
ForwardLeft = 18
BackwardLeft = 19
LeftPWM = 20


GPIO.setmode(GPIO.BOARD)
GPIO.setup(ForwardRight, GPIO.OUT)
GPIO.setup(BackwardRight, GPIO.OUT)
GPIO.setup(ForwardLeft, GPIO.OUT)
GPIO.setup(BackwardLeft, GPIO.OUT)
GPIO.setup(RightPWM, GPIO.OUT)
GPIO.setup(LeftPWM, GPIO.OUT)
rightMotorPWM = GPIO.PWM(RightPWM, 1000)
leftMotorPWM = GPIO.PWM(LeftPWM, 1000)

# this is how you turn pin stuff  GPIO.output(pin, GPIO.HIGH)
def rightMotor(direction, speed): # This controls left motor time is how long direction
    # is a boolean with true for forward and false for backward
    # Speed determines how fast
    if direction:
        GPIO.output(ForwardRight, GPIO.HIGH)
        GPIO.output(BackwardRight, GPIO.LOW)
    else:
        GPIO.output(ForwardRight, GPIO.LOW)
        GPIO.output(BackwardRight, GPIO.HIGH)
    rightMotorPWM.start(speed)


# same as RightMotor but for LeftMotor
def leftMotor(direction, speed):
    if direction:
        GPIO.output(ForwardLeft, GPIO.HIGH)
        GPIO.output(BackwardLeft, GPIO.LOW)
    else:
        GPIO.output(ForwardLeft, GPIO.LOW)
        GPIO.output(BackwardLeft, GPIO.HIGH)
    leftMotorPWM.start(speed)


print("Enter control as \"command speed time \"")
check = True
while check:
    control = input("Enter control")
    if control.lower() == "stop":
        check = False
    elif control.lower() == "help":
        print("This is help")
    else:
        commands = control.split(" ")
        try:
            speed = commands[1]
        except:
            print("you suck")
            continue
        try:
            waiting = commands[2]
        except:
            print("you suck")
            continue
        if commands[0].lower() == "forward":
            rightMotor(True, speed)
            leftMotor(True, speed)
        elif commands[0].lower() == "back":
            rightMotor(False, speed)
            leftMotor(False, speed)
        elif commands[0].lower() == "right":
            rightMotor(False, speed)
            leftMotor(True, speed)
        elif commands[0].lower() == "left":
            rightMotor(True, speed)
            leftMotor(True, speed)
        time.sleep(waiting)
        rightMotorPWM.stop()
        leftMotorPWM.stop()
GPIO.cleanup()
