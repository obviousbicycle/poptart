#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
#import roslib; roslib.load_manifest('ardrone_tutorials')
import roslib; roslib.load_manifest('irobot_mudd')
import rospy

# Import things relevant to subscribing directly to Create topics (?)
import irobot_mudd
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import time
import math

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# The GUI libraries
from PySide import QtCore, QtGui
import cv2
import numpy as np
import random


# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state

D.chargeLevel = ""


class DroneVideoDisplay(QtGui.QMainWindow):
    def __init__(self):
        # Construct the parent class
        super(DroneVideoDisplay, self).__init__()

        self.IMNUM = 1

        # Setup our very basic GUI - a label which fills the whole window and holds our image
        self.setWindowTitle('robot box')
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)
        
        # Holds the image frame received from the drone and later processed by the GUI
        self.image = None
        self.imageLock = Lock()

        self.tags = []
        self.tagLock = Lock()
        
        # Holds the status message to be displayed on the next GUI update
        self.statusMessage = ''

        # Tracks whether we have received data since the last connection check
        # This works because data comes in at 50Hz but we're checking for a connection at 4Hz
        self.communicationSinceTimer = False
        self.connected = False

        # A timer to check whether we're still connected
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
        
        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

        # get one image from file...
        fname = "./image" + str(self.IMNUM) + ".png"
        self.image2 = cv2.imread( fname )

        

    # Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback,
    # will assume we are having network troubles and display a message in the status bar
    def ConnectionCallback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

    def RedrawCallback(self):
        """ Where the image is drawn (I think) """
        global D

        if True:
            # We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
            self.imageLock.acquire()
            try:
                # if False:
                #         DATA = self.image.data
                #         WIDTH = self.image.width
                #         HEIGHT = self.image.height
                        
                #         # Convert the ROS image into a QImage which we can display
                #         image = QtGui.QPixmap.fromImage(QtGui.QImage(DATA,
                #                                                      WIDTH, HEIGHT,
                #                                                      QtGui.QImage.Format_RGB888))

                if True:
                    a = cv2.cvtColor(self.image2, cv2.COLOR_BGR2RGB)
                    WIDTH = a.shape[1]
                    HEIGHT = a.shape[0]
                    bytesPerComp = a.shape[2]
                    """
                    WIDTH = 100
                    HEIGHT = 100
                    a = np.random.randint(0,256,size=(HEIGHT,WIDTH,3))
                    print "type(a) is", type(a)
                    a = a.astype(np.uint32)
                    DATA = (255 << 24 | a[:,:,2] << 16 | a[:,:,1] << 8 | a[:,:,0]) #.flatten() 
                    WIDTH = a.shape[1]
                    HEIGHT = a.shape[0]
                    """
                    BYTESPERLINE = bytesPerComp*WIDTH
                    DATA = a.data
                    # below: PySide.
                    # what is the difference between QtGui.QImage and QtGui.QPixmap?
                    image = QtGui.QPixmap.fromImage(\
                             QtGui.QImage(DATA, WIDTH, HEIGHT, BYTESPERLINE, QtGui.QImage.Format_RGB888))

                tag = D.chargeLevel
                self.tags = [ tag ]
                        
                if len(self.tags) > 0:
                    self.tagLock.acquire()
                    try:
                        # THIS IS WHERE WE DRAW THINGS
                        # (updated every frame)
                        painter = QtGui.QPainter()
                        painter.begin(image)
                        painter.setPen(QtGui.QColor(0,0,42))
                        painter.setBrush(QtGui.QColor(0,0,42))
                        painter.drawText(10,10,'test')
                        for string_from_tag in self.tags:
                            r = QtCore.QRectF(42,142,
                                              DETECT_RADIUS*2,DETECT_RADIUS*2)
                            painter.drawEllipse(r)
                            painter.drawText(100, 100,
                                             string_from_tag)
                        painter.end()
                    finally:
                        self.tagLock.release()
            finally:
                self.imageLock.release()

            # We could  do more processing (eg OpenCV) here if we wanted to,
            # but for now lets just display the window.
            self.resize(image.width(),image.height())
            self.imageBox.setPixmap(image)

        # Update the status bar to show the current drone status & battery level
        self.statusBar().showMessage("Charge: " + D.chargeLevel)

    def ReceiveImage(self,data):
        # Indicate that new data has been received (thus we are connected)
        self.communicationSinceTimer = True
        self.IMNUM = 1
        print "in ReceiveImage"

        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        try:
            #self.image = data # Save the ros image for processing by the display thread
            fname = "./image" + str(self.IMNUM) + ".png"
            self.image2 = cv2.imread( fname )
            self.image = self.image2
            #print "type of image2 is", type(self.image2)
            #print "dir of image2 is", dir(self.image2)
            #print "self.image2.shape is", self.image2.shape
        finally:
            self.imageLock.release()

    def ReceiveNavdata(self,navdata):
        """ not much happening """
        print "In ReceiveNavdata"

def sensor_callback( data ):
    """ sensor_callback is called for each sensorPacket message
    """
    global D

    D.chargeLevel = str(int(data.chargeLevel * 100)) + '%'

def something():
    print 'hi'

if __name__=='__main__':
    import sys
    rospy.init_node('ardrone_video_display')

    # set up a callback for the sensorPacket stream, i.e., "topic"
    rospy.Subscriber( 'sensorPacket', SensorPacket, sensor_callback )

    app = QtGui.QApplication(sys.argv)
    display = DroneVideoDisplay()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
