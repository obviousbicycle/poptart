#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
import cv_bridge
import cv
import sensor_msgs.msg as sm
import time
import random

import os
import sys
import usb.core

from std_msgs.msg import String
from ControllerParser import update_D
from irobot_mudd.srv import *
from irobot_mudd.msg import *


###################### GLOBAL SYSTEM STATE ########################

# class for a generic data holder
class Data:
    def __init__(self): pass    # empty constructor
    
# object (of class Data) to hold the global state of the system
D = Data()

# do we want to use a half-sized image?
D.half_size = False

# the game controller's buttons and pads ~ initial values...
D.BTNS = [0]*16
D.PADS = [0]*16

# initialize other global variables
D.model_pixel = [0]*25
D.area = 0
D.armed = False
D.target=[0,0]
D.cenx=370
D.ceny=410
D.seeker=0

# a class to handle communicatinos with the launcher
class launchControl():
   
    def __init__(self):
        self.connectToLauncher();
        self.CODE_UP = [0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_DOWN = [0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_LEFT = [0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_RIGHT = [0x02,0x08,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_STOP = [0x02,0x20,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_FIRE = [0x02,0x10,0x00,0x00,0x00,0x00,0x00,0x00]

    def connectToLauncher(self):
      self.launcher = usb.core.find(idVendor=0x2123, idProduct=0x1010)
      if self.launcher is None:
         raise ValueError('Launcher not found.')
      if self.launcher.is_kernel_driver_active(0) is True:
         self.launcher.detach_kernel_driver(0)
      self.launcher.set_configuration()
      
    def sendCodeToLauncher(self, directive):
        if directive == "up": code = self.CODE_UP
        elif directive == "down": code = self.CODE_DOWN
        elif directive == "left": code = self.CODE_LEFT
        elif directive == "right": code = self.CODE_RIGHT
        elif directive == "fire": code = self.CODE_FIRE
        elif directive == "stop": code = self.CODE_STOP
        else:
            print "Error: unrecognized directive in sendCodeToLauncher:", directive
            code = None

        # if everything is OK, send the code to the missile launcher
        if code != None:
            self.launcher.ctrl_transfer(0x21,0x09,0,0,code)

# let's make aour launcher part of our available state...
D.launchcontrol = launchControl()


#################### STATE_MACHINE FUNCTIONS ######################

####
# Helper function to check for stops during state machine transitions
####

# initially, the FSM is NOT running, so D.STOP == True
D.STOP = True    # this will be set to False when 'F' is pressed
D.state = "none"

def transition( time, next_state_fn ):
    """ time is a float, next_state_fn is a state/function """
    global D     
    if D.STOP == True:   # check if we've been told to stop
        print "Stopping the FSM (in the transition function)"
        return  # stops the FSM ~ by not continuing it below...
    # else, we call the next state's function (next_state_fn)
    rospy.Timer( rospy.Duration(time), next_state_fn, oneshot=True )

####
# State Machine starts here
#    ... make sure each state has *only 1* call to transition
#        in each of its control branches: otherwise you get too many FSMs ...


        
####
    
def state_start( timer_event = None ):
    """ a starting state for the FSM """
    global D
    D.state = "start"
##    if 42 == 42:
##        D.elapsed_time = time.time() - D.start_fsm_time
##        print "The FSM has been going for", D.elapsed_time, "seconds."
##        transition( 1.0, state_start )
##    # these are the same for now; you may want to branch in the future...
##    else:
##        transition( 1.0, state_check)
    transition(1.0, state_check)
    
def state_check(timer_event=None):
    """check if on target"""
    global D
    D.state="check"
    precision=150
    if D.armed==False:
        D.launchcontrol.sendCodeToLauncher("stop")
        transition(.5, state_search)
    elif abs(D.target[0]-D.cenx)<=precision and abs(D.target[1]-D.ceny)<=precision:
        transition(.1, state_fire)
    else:
        transition(.1, state_aimx)

def state_aimx(timer_event = None):
    """aim at target"""
    global D
    D.state="aimx"
    precision=150
    xdiff=abs(D.target[0]-D.cenx)
    xtime=xdiff/10000.0
    #print xdiff
    #print xtime
    if D.target[0]<D.cenx-precision:
        D.launchcontrol.sendCodeToLauncher("left")
        transition(xtime, state_aimy)
    elif D.target[0]>D.cenx+precision:
        D.launchcontrol.sendCodeToLauncher("right")
        transition(xtime, state_aimy)
    else:
        D.launchcontrol.sendCodeToLauncher("stop")
        transition(.05, state_aimy)

def state_aimy(timer_event = None):
    """aim at target"""
    global D
    D.state="aimy"
    precision=150
    ydiff=abs(D.target[1]-D.ceny)
    ytime=ydiff/10000.0
    #print ytime
    if D.target[1]<D.ceny-precision:
        D.launchcontrol.sendCodeToLauncher("up")
        transition(ytime, state_check)
    elif D.target[1]>D.ceny+precision:
        D.launchcontrol.sendCodeToLauncher("down")
        transition(ytime, state_check)
    else:
        D.launchcontrol.sendCodeToLauncher("stop")
        transition(.05, state_check)    

def state_fire(timer_event = None):
    """fire at target"""
    global D
    D.state="fire"
    if D.armed==True:
        D.launchcontrol.sendCodeToLauncher("fire")
        transition(.1, state_standby)
    else:
        transition(.1, state_check)

def state_standby(timer_event = None):
    """ standby"""
    global D
    D.state="standby"
    transition(.1, state_standby)

def state_seekleft(timer_event = None):
    """ looks for object that has left the frame """
    global D
    D.state="seekleft"
    if D.area<100:
        D.launchcontrol.sendCodeToLauncher("left")
        D.seeker=D.seeker+.05
        transition(.1, state_check)
    else:
        transition(.1, state_check)
    

def state_seekright(timer_event = None):
    """ looks for object that has left the frame """
    global D
    D.state="seekright"
    if D.area<100:
        D.launchcontrol.sendCodeToLauncher("right")
        D.seeker=D.seeker+.05
        transition(.1, state_check)
    else:
        transition(.1, state_check)

def state_seekup(timer_event = None):
    """ looks for object that has left the frame """
    global D
    D.state="seekup"
    if D.area<100:
        D.launchcontrol.sendCodeToLauncher("up")
        D.seeker=D.seeker+1
        transition(.05, state_check)
    else:
        transition(.1, state_check)

def state_seekdown(timer_event = None):
    """ looks for object that has left the frame """
    global D
    D.state="seekdown"
    if D.area<100:
        D.launchcontrol.sendCodeToLauncher("down")
        D.seeker=D.seeker+1
        transition(1, state_check)
    else:
        transition(.1, state_check)   

def state_search(timer_event = None):
    """ stops the launcher if it sees the object """
    global D
    D.state="search"
    print D.seeker
    if D.seeker==0:
        transition(.1, state_seekdown)
    elif 1<=D.seeker<=2 or 5<D.seeker<=6:
        transition(.1, state_seekleft)
    elif 2<D.seeker<=3 or 4<D.seeker<=5:
        transition(.1, state_seekup)
    elif 3<D.seeker<=4 or 6<D.seeker<=7:
        transition(.1, state_seekright)
    else:
        transition(.1, state_standby)

    
    

#################### INITIALIZATION FUNCTIONS ######################
def init_globals():
    """ sets up all the globals in the dictionary D
    """
    # get D so that we can change values in it
    global D

    # put threshold values into D
    D.thresholds    = {'low_red':0, 'high_red':255,\
                       'low_green':0, 'high_green':255,\
                       'low_blue':0, 'high_blue':255,\
                       'low_hue':0, 'high_hue':255,\
                       'low_sat':0, 'high_sat':255,\
                       'low_val':0, 'high_val':255 }

    # Set up the windows containing the image from the kinect,
    # the altered image, and the threshold sliders.
    cv.NamedWindow('threshold')
    cv.MoveWindow('threshold', 400, 0)
    cv.NamedWindow('image')
    cv.MoveWindow('image', 900, 0)
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 0, 0)

    # Create the sliders within the 'sliders' window
    cv.CreateTrackbar('low_red', 'sliders', D.thresholds['low_red'], 255, 
                          lambda x: change_slider('low_red', x) )
    cv.CreateTrackbar('high_red', 'sliders', D.thresholds['high_red'], 255, 
                          lambda x: change_slider('high_red', x) )
    cv.CreateTrackbar('low_green', 'sliders', D.thresholds['low_green'], 255, 
                          lambda x: change_slider('low_green', x) )
    cv.CreateTrackbar('high_green', 'sliders', D.thresholds['high_green'], 255, 
                          lambda x: change_slider('high_green', x) )
    cv.CreateTrackbar('low_blue', 'sliders', D.thresholds['low_blue'], 255, 
                          lambda x: change_slider('low_blue', x) )
    cv.CreateTrackbar('high_blue', 'sliders', D.thresholds['high_blue'], 255, 
                          lambda x: change_slider('high_blue', x) )
    cv.CreateTrackbar('low_hue', 'sliders', D.thresholds['low_hue'], 255, 
                          lambda x: change_slider('low_hue', x) )
    cv.CreateTrackbar('high_hue', 'sliders', D.thresholds['high_hue'], 255, 
                          lambda x: change_slider('high_hue', x) )
    cv.CreateTrackbar('low_sat', 'sliders', D.thresholds['low_sat'], 255, 
                          lambda x: change_slider('low_sat', x) )
    cv.CreateTrackbar('high_sat', 'sliders', D.thresholds['high_sat'], 255, 
                          lambda x: change_slider('high_sat', x) )
    cv.CreateTrackbar('low_val', 'sliders', D.thresholds['low_val'], 255, 
                          lambda x: change_slider('low_val', x) )
    cv.CreateTrackbar('high_val', 'sliders', D.thresholds['high_val'], 255, 
                          lambda x: change_slider('high_val', x) )

    # Set the method to handle mouse button presses
    cv.SetMouseCallback('image', onMouse, None)

    # We have not created our "scratchwork" images yet
    D.created_images = False

    # Variable for key presses - set it to something that could not have been pressed
    D.last_key_pressed = 255

    # Create a connection to the Kinect
    D.bridge = cv_bridge.CvBridge()
    D.camera = cv.CaptureFromCAM(-1)
    D.running = True

    # image-based values, including the font we need to draw text
    D.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, .4, .4, 0, 1)
    D.image = None # no image yet
    D.orig_image = None

    D.pixel_delta = 10



################## END INITIALIZATION FUNCTIONS ####################

################### IMAGE PROCESSING FUNCTIONS #####################
def target_lock():
    """ sets high and low values for red, green, blue,
        hue, saturation, and value, based on the pixel
        stored in D.model_pixel, which is set whenever
        the mouse's right button is clicked
    """
    global D

    span = 30
    
    pixel_avg = [0]*6
    thresholds = [[0,0]]*6
    
    for i in range(25):
        pixel_avg[0] += D.model_pixel[i][0]
        pixel_avg[1] += D.model_pixel[i][1]
        pixel_avg[2] += D.model_pixel[i][2]
        pixel_avg[3] += D.model_pixel[i][3]
        pixel_avg[4] += D.model_pixel[i][4]
        pixel_avg[5] += D.model_pixel[i][5]

    for i in range(6):
        pixel_avg[i] = int( pixel_avg[i] / 25 )

        thresholds[i] = [pixel_avg[i] - span, pixel_avg[i] + span]

        if thresholds[i][0] > 255:
            thresholds[i][0] = 255
        elif thresholds[i][1] > 255:
            thresholds[i][1] = 255
        elif thresholds[i][0] < 0:
            thresholds[i][0] = 0
        elif thresholds[i][1] < 0:
            thresholds[i][1] = 0
    print "Here are thresholds"
    print thresholds   

    change_slider('low_red', thresholds[0][0])
    change_slider('high_red', thresholds[0][1])
    change_slider('low_green', thresholds[1][0])
    change_slider('high_green', thresholds[1][1])
    change_slider('low_blue', thresholds[2][0])
    change_slider('high_blue', thresholds[2][1])
    change_slider('low_hue', thresholds[3][0])
    change_slider('high_hue', thresholds[3][1])
    change_slider('low_sat', thresholds[4][0])
    change_slider('high_sat', thresholds[4][1])
    change_slider('low_val', thresholds[5][0])
    change_slider('high_val', thresholds[5][1])

    cv.SetTrackbarPos('low_red', 'sliders', D.thresholds['low_red'])
    cv.SetTrackbarPos('high_red', 'sliders', D.thresholds['high_red'])
    cv.SetTrackbarPos('low_green', 'sliders', D.thresholds['low_green'])
    cv.SetTrackbarPos('high_green', 'sliders', D.thresholds['high_green'])
    cv.SetTrackbarPos('low_blue', 'sliders', D.thresholds['low_blue'])
    cv.SetTrackbarPos('high_blue', 'sliders', D.thresholds['high_blue'])
    cv.SetTrackbarPos('low_hue', 'sliders', D.thresholds['low_hue'])
    cv.SetTrackbarPos('high_hue', 'sliders', D.thresholds['high_hue'])
    cv.SetTrackbarPos('low_sat', 'sliders', D.thresholds['low_sat'])
    cv.SetTrackbarPos('high_sat', 'sliders', D.thresholds['high_sat'])
    cv.SetTrackbarPos('low_val', 'sliders', D.thresholds['low_val'])
    cv.SetTrackbarPos('high_val', 'sliders', D.thresholds['high_val'])


def threshold_image():
    """ runs the image processing in order to create a 
        black and white thresholded image out of D.image
        into D.threshed_image
    """
    # get D so that we can change values in it
    global D

    # Use OpenCV to split the image up into channels,
    # saving them in their respective bw images
    cv.Split(D.image, D.blue, D.green, D.red, None)

    # This line creates a hue-saturation-value image
    cv.CvtColor(D.image, D.hsv, cv.CV_RGB2HSV)
    cv.Split(D.hsv, D.hue, D.sat, D.val, None)

    # Here is how OpenCV thresholds the images based on the slider values:
    cv.InRangeS(D.red, D.thresholds["low_red"], \
                    D.thresholds["high_red"], D.red_threshed)
    cv.InRangeS(D.green, D.thresholds["low_green"], \
                    D.thresholds["high_green"], D.green_threshed)
    cv.InRangeS(D.blue, D.thresholds["low_blue"], \
                    D.thresholds["high_blue"], D.blue_threshed)
    cv.InRangeS(D.hue, D.thresholds["low_hue"], \
                    D.thresholds["high_hue"], D.hue_threshed)
    cv.InRangeS(D.sat, D.thresholds["low_sat"], \
                    D.thresholds["high_sat"], D.sat_threshed)
    cv.InRangeS(D.val, D.thresholds["low_val"], \
                    D.thresholds["high_val"], D.val_threshed)

    # Multiply all the thresholded images into one "output" image,
    # named D.threshed_image
    cv.Mul(D.red_threshed, D.green_threshed, D.threshed_image)
    cv.Mul(D.blue_threshed, D.threshed_image, D.threshed_image)
    cv.Mul(D.hue_threshed, D.threshed_image, D.threshed_image)
    cv.Mul(D.sat_threshed, D.threshed_image, D.threshed_image)
    cv.Mul(D.val_threshed, D.threshed_image, D.threshed_image)

    # Erode and Dilate shave off and add edge pixels respectively
    #cv.Erode(D.threshed_image, D.threshed_image, iterations = 1)
    #cv.Dilate(D.threshed_image, D.threshed_image, iterations = 1)


def find_biggest_region():
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks in in the main image
    """
    # get D so that we can change values in it
    global D

    cv.Copy( D.threshed_image, D.copy ) # copy threshed image

    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(D.copy, D.storage, cv.CV_RETR_EXTERNAL, \
                                   cv.CV_CHAIN_APPROX_SIMPLE)

    # Next we want to find the *largest* contour
    if len(contours) > 0:
        biggest = contours
        biggestArea = cv.ContourArea(contours)
        while contours != None:
            nextArea = cv.ContourArea(contours)
            if biggestArea < nextArea:
                biggest = contours
                biggestArea = nextArea
            contours = contours.h_next()
        
        # Use OpenCV to get a bounding rectangle for the largest contour
        br = cv.BoundingRect(biggest, update=0)

        #print "in find_regions, br is", br

        # Example of drawing a red box
        # Variables: ulx ~ upper left x, lry ~ lower right y, etc.
        ulx = br[0]
        lrx = br[0] + br[2]
        uly = br[1]
        lry = br[1] + br[3]
        cv.PolyLine(D.image, [[(ulx,uly), (lrx,uly), 
                               (lrx,lry), (ulx,lry)]], 
                                1, cv.RGB(255, 0, 0))

        D.area = abs((ulx - lrx)*(uly - lry))
        D.target = [(ulx+lrx)/2,(uly+lry)/2]
        # Example of drawing a yellow circle
        # Variables: cenx, ceny
        #cenx = (ulx+lrx)/2
        # Crosshairs
        D.cenx=400
        #ceny = (uly+lry)/2
        D.ceny=410
        if D.armed==True:
            cv.Circle(D.image, (D.cenx,D.ceny), 8, 
                            cv.RGB(255, 255, 0), 
                            thickness=1, lineType=8, shift=0)



def draw_text_to_image():
    """ A function to handle drawing things to the image """
    # draw a rectangle under the text to make it more visible
    cv.Rectangle(D.image, (0,0), (100,80), cv.RGB(255,255,255), cv.CV_FILLED)
    # place some text there
    cv.PutText(D.image, "State: " + D.state, (5,10), D.font, cv.RGB(0,0,0))
    # and some values
    #randomness = random.randint(-100,100)
    area = D.area
    area_string = str(area)
    cv.PutText(D.image, "Area: " + area_string, (5,25), D.font, cv.RGB(0,0,0))
    # write armed/disarmed state
    if D.area>500 and D.area<2000:
        D.armed=True
    else:
        D.armed=False
    armed_string = str(D.armed)
    cv.PutText(D.image, "Armed: " + armed_string, (5,40), D.font, cv.RGB(0,0,0))
    #Distances
    xdiff=abs(D.target[0]-D.cenx)
    ydiff=abs(D.target[1]-D.ceny)
    cv.PutText(D.image, "X Diff: " + str(xdiff), (5,55), D.font, cv.RGB(0,0,0))
    cv.PutText(D.image, "Y Diff: " + str(ydiff), (5,70), D.font, cv.RGB(0,0,0))



################# END IMAGE PROCESSING FUNCTIONS ###################

####################### CALLBACK FUNCTIONS #########################
def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """

    if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
        print "x, y are", x, y
        (b,g,r) = D.image[y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v) = D.hsv[y,x]
        print "h,s,v is", int(h), int(s), int(v)

    if event==cv.CV_EVENT_RBUTTONDOWN: # clicked the right button
        print "\n\n  *** Right button click! *** \n\n"
        # you'll want to use this to start a model of the pixel
        # colors you'd like to track...
        # first, print the r,g,b and h,s,v at the right-clicked point...
        print "x, y are", x, y
        (b,g,r) = D.image[y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v) = D.hsv[y,x]
        print "h,s,v is", int(h), int(s), int(v)

        # obtain color info from a 5x5 square of pixels centered around
        # the clicked point
        n = 0
        print "D.model_pixel is:"
        print "\n"
        for i in range(-2,3):
            for j in range(-2,3):
                (b,g,r) = D.image[y+j,x+i]
                (h,s,v) = D.hsv[y+j,x+i]
                D.model_pixel[n] = [r,g,b,h,s,v]
                print D.model_pixel[n]
                n += 1
            print "\n"

        target_lock()

def check_key_press(key_press):
    """ this handler is called when a real key press has been
        detected, and updates everything appropriately
    """
    # get D so that we can change values in it
    global D
    D.last_key_pressed = key_press


    if key_press == ord('q') or key_press == 27: # if a 'q' or ESC was pressed
        print "quitting... first, stopping the FSM"
        D.STOP = True  # stop the FSM...
        time.sleep(2)  # give it some time to stop
        D.running = False  # stops the main loop
        return

    elif key_press == ord('F') or D.BTNS[0]==1:
        # toggle our FSM
        if D.STOP == True: # the FSM is not running
            print "Starting FSM..."
            D.STOP = False   # so the FSM will run...
            D.start_fsm_time = time.time()  # record our fsm's starting time...
            transition(0.1, state_start)
        else:
            D.STOP = True  # will stop one, if it's running
            print "Stopping FSM..."
            time.sleep(2.0) # wait a bit so that the FSM thread will stop

    elif key_press == ord('i') or D.PADS[1]==-1.0:
            D.launchcontrol.sendCodeToLauncher("up")
    elif key_press == ord('k') or D.PADS[1]==1.0:
            D.launchcontrol.sendCodeToLauncher("down")
    elif key_press == ord('l') or D.PADS[0]==-1.0:
            D.launchcontrol.sendCodeToLauncher("left")
    elif key_press == ord('j') or D.PADS[0]==1.0:
            D.launchcontrol.sendCodeToLauncher("right")

    elif key_press == ord('\n') or D.BTNS[2]==1:
            D.launchcontrol.sendCodeToLauncher("fire")
    elif key_press in [ord('z'), ord(' ')] or D.BTNS[1]==1:
            D.launchcontrol.sendCodeToLauncher("stop")
        
    elif key_press == ord('S'):  # save to file
        x = D.thresholds   # the value we will save
        f = open( "./thresh.txt", "w" )   # open the file "thresh.txt" for writing
        print >> f, x   # print x to the file object f
        f.close()   # it's good to close the file afterwards
        print "Saving current slider threshold values..."
        
    elif key_press == ord('L'):  # load from file
        try:
            f = open( "./thresh.txt", "r" )   # open the file "thresh.txt" for reading
            data = f.read()   # read everything from f into data
            x = eval( data )  # eval is Python's evaluation function
            f.close()   # it's good to close the file afterwards
            D.thresholds = x
            cv.SetTrackbarPos('low_red', 'sliders', D.thresholds['low_red'])
            cv.SetTrackbarPos('high_red', 'sliders', D.thresholds['high_red'])
            cv.SetTrackbarPos('low_green', 'sliders', D.thresholds['low_green'])
            cv.SetTrackbarPos('high_green', 'sliders', D.thresholds['high_green'])
            cv.SetTrackbarPos('low_blue', 'sliders', D.thresholds['low_blue'])
            cv.SetTrackbarPos('high_blue', 'sliders', D.thresholds['high_blue'])
            cv.SetTrackbarPos('low_hue', 'sliders', D.thresholds['low_hue'])
            cv.SetTrackbarPos('high_hue', 'sliders', D.thresholds['high_hue'])
            cv.SetTrackbarPos('low_sat', 'sliders', D.thresholds['low_sat'])
            cv.SetTrackbarPos('high_sat', 'sliders', D.thresholds['high_sat'])
            cv.SetTrackbarPos('low_val', 'sliders', D.thresholds['low_val'])
            cv.SetTrackbarPos('high_val', 'sliders', D.thresholds['high_val'])
            print "Loaded saved slider threshold values..."
        except:
            print "No file named thresh.txt found!"
        
        
           
    else:
        print "Did not recognize a command for key #", key_press, "(", chr(key_press), ")"


def change_slider(name, new_threshold):
    """ changes the slider values given the name of the slider and the new value """
    # get D so that we can change values in it
    global D
    D.thresholds[name] = new_threshold


def init_images():
    """ Creates all the images we'll need. Is separate from init_globals 
        since we need to know what size the images are before we can make
        them
    """
    # get D so that we can change values in it
    global D

    # Find the size of the image 
    # (we set D.image right before calling this function)
    D.img_size = cv.GetSize(D.image)

    # Create images for each color channel
    D.red = cv.CreateImage(D.img_size, 8, 1)
    D.green = cv.CreateImage(D.img_size, 8, 1)
    D.blue = cv.CreateImage(D.img_size, 8, 1)
    D.hue = cv.CreateImage(D.img_size, 8, 1)
    D.sat = cv.CreateImage(D.img_size, 8, 1)
    D.val = cv.CreateImage(D.img_size, 8, 1)

    # Create images to save the thresholded images to
    D.red_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.green_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.blue_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.hue_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.sat_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.val_threshed = cv.CreateImage(D.img_size, 8, 1)

    # Create storage for image processing later on...
    D.storage = cv.CreateMemStorage(0) # Create memory storage for contours
    D.copy = cv.CreateImage(D.img_size, 8, 1)

    # The final thresholded result
    D.threshed_image = cv.CreateImage(D.img_size, 8, 1)

    # Create the hsv image
    D.hsv = cv.CreateImage(D.img_size, 8, 3)


def handle_data():
    """ this method processes data given:
        - key presses
        - images from webcam
    """
    # get D so that we can change values in it
    global D

    while D.running:

        # Grab incoming image
        D.orig_image = cv.QueryFrame(D.camera)
        # handle half_size
        if D.half_size == False: # keep the image size
            D.image = D.orig_image
        else: # halve the image size
            if D.created_images == False:
                w, h = D.orig_image.width, D.orig_image.height
                D.half_sz = (w/2,h/2)
                D.image = cv.CreateImage(D.half_sz, 8, 3)
            cv.Resize(D.orig_image,D.image)

        # Get the incoming image from the Kinect
        #D.image = D["bridge"].imgmsg_to_cv(data, "bgr8")

        if D.created_images == False:
            # Initialize the additional images we need for processing
            # We only need to run this one time
            init_images()
            D.created_images = True

        # Recalculate threshold image
        threshold_image()

        # Recalculate blob in main image
        find_biggest_region()

        # draw any text on the image
        draw_text_to_image()

        # Get any incoming keypresses
        # To get input from keyboard, we use cv.WaitKey
        # Only the lowest eight bits matter (so we get rid of the rest):
        key_press_raw = cv.WaitKey(5) # gets a raw key press
        key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
        
        # Handle key presses only if it's a real key (255 = "no key pressed")
        if key_press != 255:  check_key_press(key_press)

        # Update the displays:
        # Main image:
        cv.ShowImage('image', D.image)

        # Currently selected threshold image:
        cv.ShowImage('threshold', D.threshed_image )

    # here, the main loop has ended
    print "Shutting down..."


def controller_callback(data): #ADDED
    global D
    message = data.data
    update_D(D, message)
    # print the PADS and BTNS
    print "PADS is", D.PADS
    print "BTNS is", D.BTNS
    check_key_press(255)

##################### END CALLBACK FUNCTIONS #######################

def main():
    """ the main program that sets everything up
    """
    
    # Initialize our node
    rospy.init_node('blobFinder')

    # here is the subscriber to controller data...
    rospy.Subscriber( 'controller_data', String, controller_callback ) #ADDED

    # Initialize all the global variables we will need
    init_globals()

    # Call the function that begins our process
    handle_data()


# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
    main()
