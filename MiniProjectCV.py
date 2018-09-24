import cv2
from picamera import PiCamera
import picamera.array
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
from fractions import Fraction

global red_low, red_high, green, SBD

red_low = np.array([[0, 120, 50],[10, 255, 255]])
red_high = np.array([[170, 120, 50], [179, 255, 255]])
green = np.array([[50, 50, 50], [65, 255, 255]])

params_SBD = cv2.SimpleBlobDetector_Params() # Setting up the simple blob detector parameters object
params_SBD.filterByCircularity = True # Turning on filter by circularity
params_SBD.minCircularity = 0.7
params_SBD.maxCircularity = 1 # Setting minimum circularity
params_SBD.filterByArea = False # Turning off filter by area
params_SBD.filterByConvexity = False # Turning off filter by convexity
params_SBD.filterByInertia = False # Turning off filter by Intertia
params_SBD.filterByColor = False # Turning off filter by color
SBD = cv2.SimpleBlobDetector_create(params_SBD) # Creating the simple blob detector


def isolateLED(camera):
    image = picamera.array.PiRGBArray(camera) # Define Image
    camera.capture(image, format='rgb') # Take picture
    imageData = image.array # Get array data

    imageData = imageData[...,::-1] # Converts RGB to BGR
    imageData = cv2.cvtColor(imageData, cv2.COLOR_BGR2HSV) # Converts to HSV



    mask_rl = np.array(cv2.inRange(imageData, red_low[0], red_low[1]))
    mask_rh = np.array(cv2.inRange(imageData, red_high[0], red_high[1]))
    mask_g = np.array(cv2.inRange(imageData, green[0], green[1]))

    mask_rr = mask_rl | mask_rh
    
    mask_t = mask_rl | mask_rh | mask_g

    result = cv2.bitwise_and(imageData, imageData, mask=mask_t)
    result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)

    kernel = np.ones((5,5), np.uint8) # Creates the kernel that will be used for morphological opening and closing
    result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel) # Opens the image
    resut = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel) # Closes the image
    
    
    kernel = np.ones((10,10), np.uint8) # Creates the kernel that will be used for morphological opening and closing

    result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel) # Opens the image
    resut = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel) # Closes the image
    result = cv2.blur(result, (1, 1))

    return result


def detColor(image):
    orgImage = image
    ret, image = cv2.threshold(image, 50, 255, cv2.THRESH_BINARY) # Converts the image to a binary image
    keypoints = SBD.detect(image) # Getting the keypoints of the detected blobs in the image
    drawn_keypoints = cv2.drawKeypoints(orgImage, keypoints, np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    if (len(keypoints) > 0):
        pixel = orgImage[round(keypoints[0].pt[1]), round(keypoints[0].pt[0])]
        if (len(keypoints) >= 2):
            color = "More than one color detected"
        elif (pixel[2] > pixel[1]):
            color = "red"
        elif (pixel[2] < pixel[1]):
            color = "green"
    else:
        color = "NA"
    
    
    
    
    return color



# Camera setup
camera = PiCamera()
camera.resolution = (736, 400)
camera.iso = 1
sleep(2)

camera.shutter_speed = 5000
camera.exposure_mode = 'off'
g = (39/32, 209/128)
camera.awb_mode = 'off'
camera.awb_gains = (Fraction(39, 32), Fraction(209, 128))

#camera.start_preview()
#sleep(1)
#camera.stop_preview()

    
while(True):
    masked = isolateLED(camera)
    color = detColor(masked)
    print(color, "\n")









    #userIn = input('Continue? (y/n)')
    #if (userIn == 'y'):
     #   continue
    #elif (userIn == 'n'):
    #    break  
    
    
