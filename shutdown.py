#!/usr/bin/env python
# http://www.raspberry-pi-geek.com/Archive/2013/01/Adding-an-On-Off-switch-to-your-Raspberry-Pi/(offset)/4
# Import the modules to send commands to the system and access GPIO pins
from subprocess import call
import RPi.GPIO as gpio

# Define a function to keep script running
def loop():
    raw_input()

# Define a function to run when an interrupt is called
def shutdown(pin):
    call('halt', shell=False)

gpio.setmode(gpio.BCM) # Set pin numbering to board numbering
gpio.setup(9, gpio.IN) # Set up pin 7 as an input
gpio.add_event_detect(9, gpio.FALLING, callback=shutdown, bouncetime=200) # Set up an interrupt to look for button presses

loop() # Run the loop function to keep script running
