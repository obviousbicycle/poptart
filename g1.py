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
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state

D.chargeLevel = ""

D.x = ""
D.y = ""
D.theta = ""


class RobotBox(QtGui.QMainWindow):
    def __init__(self):
        # Construct the parent class
        super(RobotBox, self).__init__()

        self.IMNUM = 1

        #####
        # Central Widget: imageBox
        #####

        # Setup our very basic GUI - a label which fills the whole window and holds our image
        self.setWindowTitle('RobotBox')
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)
        
        # Holds the image frame received from the drone and later processed by the GUI
        self.image = None
        self.imageLock = Lock()

        self.tags = []
        self.tagLock = Lock()
        
        # Holds the status message to be displayed on the next GUI update
        self.statusMessage = ''
        
        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

        # get one image from file...
        fname = "./image" + str(self.IMNUM) + ".png"
        self.image2 = cv2.imread( fname )

        #####
        # Left Dock Widget: infoBox
        #####

        # infoBox is just a generic widget holding a grid layout
        self.infoBox = QtGui.QWidget(self)

        # Save/load image buttons
        self.saveImage = QtGui.QPushButton("&Save")
        self.loadImage = QtGui.QPushButton("&Load")

        # Position info
        xLabel = QtGui.QLabel(self)
        xLabel.setText("x: ")
        self.xValue = QtGui.QLabel(self)
        self.xValue.setText("")

        yLabel = QtGui.QLabel(self)
        yLabel.setText("y: ")
        self.yValue = QtGui.QLabel(self)
        self.yValue.setText("")

        thetaLabel = QtGui.QLabel(self)
        thetaLabel.setText("theta: ")
        self.thetaValue = QtGui.QLabel(self)
        self.thetaValue.setText("")

        # Putting together layout and dock widget
        infoLayout = QtGui.QGridLayout()
        self.infoBox.setLayout(infoLayout)

        infoLayout.addWidget(xLabel, 0, 0, QtCore.Qt.AlignRight)
        infoLayout.addWidget(self.xValue, 0, 1)
        infoLayout.addWidget(yLabel, 1, 0, QtCore.Qt.AlignRight)
        infoLayout.addWidget(self.yValue, 1, 1)
        infoLayout.addWidget(thetaLabel, 2, 0, QtCore.Qt.AlignRight)
        infoLayout.addWidget(self.thetaValue, 2, 1)
        infoLayout.addWidget(self.saveImage, 3, 0)
        infoLayout.addWidget(self.loadImage, 3, 1)

        self.infoDock = QtGui.QDockWidget("", self)
        self.infoDock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
        self.infoDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.infoDock.setTitleBarWidget(QtGui.QWidget()) # no title
        self.infoDock.setWidget(self.infoBox)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.infoDock)

    def RedrawCallback(self):
        global D

        self.imageLock.acquire()
        try:
            # a = cv2.cvtColor(self.image2, cv2.COLOR_BGR2RGB)
            # WIDTH = a.shape[1]
            # HEIGHT = a.shape[0]
            # bytesPerComp = a.shape[2]
            # BYTESPERLINE = bytesPerComp*WIDTH
            # DATA = a.data
            # # below: PySide.
            # # what is the difference between QtGui.QImage and QtGui.QPixmap?
            # image = QtGui.QPixmap.fromImage(\
            #          QtGui.QImage(DATA, WIDTH, HEIGHT, BYTESPERLINE, QtGui.QImage.Format_RGB888))

            # Default image is just a white square
            blank = QtGui.QPixmap.fromImage(\
                QtGui.QImage(300, 300, QtGui.QImage.Format_RGB888))
            blank.fill(QtGui.QColor(255,255,255))

            image = blank

            self.xValue.setText(D.x)
            self.yValue.setText(D.y)
            self.thetaValue.setText(D.theta)

            tag = D.chargeLevel
            self.tags = [ tag ]
                    
            if len(self.tags) > 0:
                self.tagLock.acquire()
                try:
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


def sensor_callback( data ):
    """ sensor_callback is called for each sensorPacket message
    """
    global D

    D.chargeLevel = str(int(round(data.chargeLevel * 100))) + '%'

    D.x = '%.6f' % data.x
    D.y = '%.6f' % data.y
    D.theta = '%.6f' % data.theta


if __name__=='__main__':
    import sys
    rospy.init_node('RobotBox')

    # set up a callback for the sensorPacket stream, i.e., "topic"
    rospy.Subscriber( 'sensorPacket', SensorPacket, sensor_callback )

    app = QtGui.QApplication(sys.argv)
    display = RobotBox()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
