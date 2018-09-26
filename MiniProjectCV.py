# -*- coding: utf-8 -*-
#!/usr/bin/python
import cv2
from picamera import PiCamera
import picamera.array
import numpy as np
from time import sleep
import smbus
from fractions import Fraction
import FunctionsForMiniProject as ffmp

#import Adafruit_CharLCD as LCD
#Initialize LCD screen
#lcd = LCD.Adafruit_CharLCDPlate()


# Camera setup
camera = PiCamera() # Initialising camera object
camera.resolution = (320, 170) # Setting camera resolution (lower for speed)
camera.iso = 1 # Setting iso
sleep(2) # Letting camera "warm up"

camera.shutter_speed = 5000 # Setting shutter speed (low so LEDs are not white
camera.exposure_mode = 'off' # Turining off exposure mode
camera.awb_mode = 'off' # Turning auto white balance off for consistancy
camera.awb_gains = (Fraction(39, 32), Fraction(209, 128)) # Setting autowhite balance gains
    
while(True):
    masked = ffmp.isolateLED(camera) # mask the image to isolate the red and green 
    # cv2.imshow('Masked Image', masked) # Displays the masked image at every iteration
    # cv2.waitKey(0)
    color = ffmp.detColor(masked) # Determine the color LED (if any)
    print(color, "\n") # Print out the color found
    ffmp.writeNumber(color) 
    #Speed = ffmp.readNumber()
    #lcd.clear()
    #lcd.message("swag")
    
    
