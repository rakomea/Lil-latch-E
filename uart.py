#!/usr/bin/env python
 
import time         # time library for delays
import serial       # library for UART serial interfacing
from inputs import devices  # dependency for the peripherial devices
from inputs import get_gamepad  # dependency for gamepad interfacing

gamepad = devices.gamepads[0]   # initialize the gamepad device object from the device vector

serial_interface = serial.Serial('/dev/ttyAMA0',9600)   # initilaize the serial interface object for UART
print(serial_interface.name)    # console output

# Function that convets raw gamepad values message byte
def gp2dirX(gp):
    if gp < 50:     
        return 'L'  # return 'L' for left
    elif gp >= 50 and gp <= 205:
        return 'C'  # return 'C' for center
    elif gp > 205:
        return 'R'  # return 'R' for right

# Function that convets raw gamepad values message byte
def gp2dirY(gp):
    if gp < 50:
        return 'F'  # return 'F' for forward
    elif gp >= 50 and gp <= 205:
        return 'C'  # return 'C' for center
    elif gp > 205:
        return 'B'  # return 'B' for backward

delay = 0.01;   # time delays after uart transmission 
# infinite main loop 
while True: 
    gamepad_events = gamepad.read() # read the gamepad inptus
    for gamepad_event in gamepad_events:    # iterate through the event codes sent by the gamepad
        code = gamepad_event.code           # temp var contains code
        state = gamepad_event.state         # temp var contains state
        if code == "ABS_X":                 # X-coordinate data
            serial_interface.write('X')     # write to UART the domain
            time.sleep(delay)               # delay
            serial_interface.write(gp2dirX(state))  # write to UART the direction
            print('X:' + gp2dirX(state))    # print to console for debugging
        if gamepad_event.code == "ABS_Y":   # Y-coordinate data
            serial_interface.write('Y')     # write to UART the domain
            time.sleep(delay)               # delay
            serial_interface.write(gp2dirY(state))  # write to UART the direction
            print('Y:' + gp2dirY(state))    # print to console for debugging
        time.sleep(delay)                   # delay
        serial_interface.write('\n');       # write to UART: endline char to mark the end
        time.sleep(delay)                   # delay

serialinterface.close()     # closes the UART serial connection (will theoretically never fire)
