import cv2
from picamera import PiCamera
import picamera.array
import numpy as np
from time import sleep
import smbus
from fractions import Fraction

global red_low, red_high, green, SBD

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04


red_low = np.array([[0, 120, 50],[10, 255, 255]]) # Lower section for red HSV values
red_high = np.array([[170, 120, 50], [179, 255, 255]]) # Upper section for red HSV values
green = np.array([[50, 50, 50], [65, 255, 255]]) # Accepted HSV values for green

params_SBD = cv2.SimpleBlobDetector_Params() # Setting up the simple blob detector parameters object
params_SBD.filterByCircularity = True # Turning on filter by circularity
params_SBD.minCircularity = 0.7 # Minimum circularity that will be accepted (rest filtered out)
params_SBD.maxCircularity = 1 # Setting minimum circularity
params_SBD.filterByArea = False # Turning off filter by area
params_SBD.filterByConvexity = False # Turning off filter by convexity
params_SBD.filterByInertia = False # Turning off filter by Intertia
params_SBD.filterByColor = False # Turning off filter by color
SBD = cv2.SimpleBlobDetector_create(params_SBD) # Creating the simple blob detector

def writeNumber(value):
    bus.write_byte(address, value)
# bus.write_byte_data(address, 0, value)
    return -1


def isolateLED(camera): # Masks the image to isolate the LEDs
    image = picamera.array.PiRGBArray(camera) # Define Image
    camera.capture(image, format='rgb') # Take picture
    imageData = image.array # Get array data

    imageData = imageData[...,::-1] # Converts RGB to BGR
    imageData = cv2.cvtColor(imageData, cv2.COLOR_BGR2HSV) # Converts to HSV



    mask_rl = np.array(cv2.inRange(imageData, red_low[0], red_low[1])) # Lower red mask
    mask_rh = np.array(cv2.inRange(imageData, red_high[0], red_high[1])) # Upper red mask
    mask_g = np.array(cv2.inRange(imageData, green[0], green[1])) # Lower green mask
    
    mask_t = mask_rl | mask_rh | mask_g # Combining all masks

    result = cv2.bitwise_and(imageData, imageData, mask=mask_t) # Masking image
    result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR) # Converting from HSV to RGB

    kernel = np.ones((3,3), np.uint8) # Creates the kernel that will be used for morphological opening and closing
    result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel) # Opens the image
    resut = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel) # Closes the image
    
    
    kernel = np.ones((5,5), np.uint8) # Creates the kernel that will be used for morphological opening and closing

    result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel) # Opens the image
    resut = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel) # Closes the image
    result = cv2.blur(result, (2, 2)) # Bluring image to make LEDs more consistantly valued across

    return result


def detColor(image): # Determines the color of the LED (if any) in the image
    orgImage = image # Saving the original image
    ret, image = cv2.threshold(image, 75, 255, cv2.THRESH_BINARY) # Converts the image to a binary image
    keypoints = SBD.detect(image) # Getting the keypoints of the detected blobs in the image
    # drawn_keypoints = cv2.drawKeypoints(orgImage, keypoints, np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) # Drawing the keypoints on the image
    if (len(keypoints) > 0): # If blobs detected, determine the color
        pixel = orgImage[round(keypoints[0].pt[1]), round(keypoints[0].pt[0])]
        if (len(keypoints) >= 2): # If there are 2 or more blobs detected, can't determine the color
            return 2
        elif (pixel[2] > pixel[1]): # If red value in BGR is higher than the green value, LED is red
            return 0
        elif (pixel[2] < pixel[1]): # If the green value in BGR is higher than the red value, LED is green
            return 1
    else: # If there are no blobs detected, no LED present
        return 2
    
    return 2
