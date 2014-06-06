#!/usr/bin/env python

# A window displaying the Create's movements based on its odometry

# Import the ROS libraries, and load the manifest file which through
# <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('irobot_mudd')
import rospy

# Import things relevant to subscribing directly to Create topics (?)
import irobot_mudd
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import time
import math

# We need to use resource locking to handle synchronization between GUI thread
# and ROS topic callbacks
from threading import Lock

# The GUI libraries
from PySide import QtCore, QtGui
from PySide.QtCore import Slot
import cv2
import numpy as np
import random


# Some Constants
GUI_UPDATE_PERIOD = 20 # ms
USE_CM = 1 # set to 0 to use meters in position data
DRAW_TRAIL = True # experimental feature that can be easily turned off


class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state

D.chargeLevel = ""

# for image loading
D.loadedImage = QtGui.QImage()

# holds robot trail
D.trail = []

# in meters/degrees
D.x = 0.0
D.y = 0.0
D.theta = 0.0

# for "clearing" odometer
D.xDiff = 0.0
D.yDiff = 0.0
D.thetaDiff = 0.0

# IR light sensor data
D.backLeftSensor = 0
D.frontLeftSensor = 0
D.frontRightSensor = 0
D.backRightSensor = 0


class RobotBox(QtGui.QMainWindow):
    def __init__(self):
        # Construct the parent class
        super(RobotBox, self).__init__()

        self.initCentral()
        self.initLeft()
        self.initBottom()
        self.initRight()

    def initCentral(self):
        """
        Central Widget: imageBox
        Displays top-down map over the robot
        """
        # Setup our very basic GUI - a label which fills the whole window and
        # holds our image
        self.setWindowTitle('RobotBox')
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)
        
        # Holds the image frame received from the drone and later processed by
        # the GUI
        self.image = None
        self.imageLock = Lock()

        # Holds the status message to be displayed on the next GUI update
        self.statusMessage = ''
        
        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

    def initLeft(self):
        """
        Left Dock Widget: positionBox
        Displays position data from sensorPacket
        """
        # positionBox is just a generic widget holding a grid layout
        self.positionBox = QtGui.QWidget(self)

        # Position info + reset buttons
        xLabel = QtGui.QLabel(self)
        xLabel.setText("x (" + USE_CM*"c" + "m): ")
        self.xValue = QtGui.QLabel(self)
        self.xValue.setText("")
        xResetButton = QtGui.QPushButton("Reset", self)
        xResetButton.clicked.connect(self.XReset)

        yLabel = QtGui.QLabel(self)
        yLabel.setText("y (" + USE_CM*"c" + "m): ")
        self.yValue = QtGui.QLabel(self)
        self.yValue.setText("")
        yResetButton = QtGui.QPushButton("Reset", self)
        yResetButton.clicked.connect(self.YReset)

        thetaLabel = QtGui.QLabel(self)
        thetaLabel.setText("theta (deg): ")
        self.thetaValue = QtGui.QLabel(self)
        self.thetaValue.setText("")
        thetaResetButton = QtGui.QPushButton("Reset", self)
        thetaResetButton.clicked.connect(self.ThetaReset)

        allResetButton = QtGui.QPushButton("Reset all", self)
        allResetButton.clicked.connect(self.AllReset)

        # Putting together layout and dock widget
        positionLayout = QtGui.QGridLayout()
        self.positionBox.setLayout(positionLayout)

        positionLayout.addWidget(xLabel, 0, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.xValue, 0, 1)
        positionLayout.addWidget(xResetButton, 1, 1)
        positionLayout.addWidget(yLabel, 2, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.yValue, 2, 1)
        positionLayout.addWidget(yResetButton, 3, 1)
        positionLayout.addWidget(thetaLabel, 4, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.thetaValue, 4, 1)
        positionLayout.addWidget(thetaResetButton, 5, 1)
        positionLayout.addWidget(allResetButton, 6, 1)

        positionTitle = QtGui.QLabel(self)
        positionTitle.setText("Position:")
        positionTitle.setAlignment(QtCore.Qt.AlignHCenter)

        self.positionDock = QtGui.QDockWidget("", self)
        self.positionDock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
        self.positionDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.positionDock.setTitleBarWidget(positionTitle)
        self.positionDock.setWidget(self.positionBox)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.positionDock)

    def initBottom(self):
        """
        Bottom Dock Widget: lightBox
        Displays IR light sensor data from sensorPacket
        """
        # lightBox is just a generic widget holding a grid layout
        self.lightBox = QtGui.QWidget(self)

        # Light sensor info
        backLeftLabel = QtGui.QLabel(self)
        backLeftLabel.setText("Back left: ")
        self.backLeftValue = QtGui.QLabel(self)
        self.backLeftValue.setText("")

        frontLeftLabel = QtGui.QLabel(self)
        frontLeftLabel.setText("Front left: ")
        self.frontLeftValue = QtGui.QLabel(self)
        self.frontLeftValue.setText("")

        frontRightLabel = QtGui.QLabel(self)
        frontRightLabel.setText("Front right: ")
        self.frontRightValue = QtGui.QLabel(self)
        self.frontRightValue.setText("")

        backRightLabel = QtGui.QLabel(self)
        backRightLabel.setText("Back right: ")
        self.backRightValue = QtGui.QLabel(self)
        self.backRightValue.setText("")

        # Putting together layout and dock widget
        lightLayout = QtGui.QGridLayout()
        self.lightBox.setLayout(lightLayout)

        lightLayout.addWidget(backLeftLabel, 0, 0, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backLeftValue, 1, 0,\
            QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(frontLeftLabel, 0, 1, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftValue, 1, 1,\
            QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(frontRightLabel, 0, 2, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightValue, 1, 2,\
            QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(backRightLabel, 0, 3, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightValue, 1, 3,\
            QtCore.Qt.AlignHCenter)

        lightTitle = QtGui.QLabel(self)
        lightTitle.setText("Light sensors:")
        lightTitle.setAlignment(QtCore.Qt.AlignHCenter)

        self.lightDock = QtGui.QDockWidget("", self)
        self.lightDock.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea)
        self.lightDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.lightDock.setTitleBarWidget(lightTitle)
        self.lightDock.setWidget(self.lightBox)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self.lightDock)

    def initRight(self):
        """
        Right Dock Widget: actionBox
        Displays various buttons of interest (save, load, clear)
        """
        # actionBox is just a generic widget holding a grid layout
        # Note: the grid layout is currently unnecessary, but if we expand it
        # later such that it then requires a grid, it'll be less of a headache.
        self.actionBox = QtGui.QWidget(self)

        # Image action buttons
        self.saveButton = QtGui.QPushButton("&Save image", self)
        self.saveButton.clicked.connect(self.SaveImage)

        self.loadButton = QtGui.QPushButton("&Load image", self)
        self.loadButton.clicked.connect(self.LoadImage)

        self.clearButton = QtGui.QPushButton("Clear image", self)
        self.clearButton.clicked.connect(self.ClearImage)

        self.eraseButton = QtGui.QPushButton("Erase trail", self)
        self.eraseButton.clicked.connect(self.EraseTrail)

        # Putting together layout and dock widget
        actionLayout = QtGui.QGridLayout()
        self.actionBox.setLayout(actionLayout)

        actionLayout.addWidget(self.saveButton, 0, 0)
        actionLayout.addWidget(self.loadButton, 1, 0)
        actionLayout.addWidget(self.clearButton, 2, 0)
        actionLayout.addWidget(self.eraseButton, 3, 0)

        actionTitle = QtGui.QLabel(self)
        actionTitle.setText("Actions:")
        actionTitle.setAlignment(QtCore.Qt.AlignHCenter)

        self.actionDock = QtGui.QDockWidget("", self)
        self.actionDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        self.actionDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.actionDock.setTitleBarWidget(actionTitle)
        self.actionDock.setWidget(self.actionBox)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.actionDock)

    def RedrawCallback(self):
        global D

        self.imageLock.acquire()
        try:
            # updating infoBox values
            # if USE_CM = 1, values are converted here
            xDisplay = round(100.0**USE_CM * (D.x - D.xDiff), 2)
            yDisplay = round(100.0**USE_CM * (D.y - D.yDiff), 2)
            self.xValue.setText(str(xDisplay))
            self.yValue.setText(str(yDisplay))

            # convert theta to degrees and set bounds
            thetaDisplay = round(math.degrees(D.theta - D.thetaDiff))
            while thetaDisplay > 360.0:
                thetaDisplay -= 360.0
            while thetaDisplay < 0.0:
                thetaDisplay += 360.0
            self.thetaValue.setText(str(int(thetaDisplay)))

            # updating lightBox values
            self.backLeftValue.setText(str(D.backLeftSensor))
            self.frontLeftValue.setText(str(D.frontLeftSensor))
            self.frontRightValue.setText(str(D.frontRightSensor))
            self.backRightValue.setText(str(D.backRightSensor))

            # updating imageBox
            # right now, it just redraws the entire image every time the GUI is
            # updated...
            if D.loadedImage.isNull():
            # Default image is just a white square
                WIDTH = 300
                HEIGHT = 300
                blank = QtGui.QPixmap.fromImage(\
                    QtGui.QImage(WIDTH, HEIGHT, QtGui.QImage.Format_RGB888))
                blank.fill(QtGui.QColor(255,255,255))

                image = blank
            else:
                WIDTH = D.loadedImage.width()
                HEIGHT = D.loadedImage.height()
                image = QtGui.QPixmap.fromImage(D.loadedImage)

            origin = [WIDTH/2, HEIGHT/2]
            location = (origin[0]+xDisplay,origin[1]-yDisplay)

            painter = QtGui.QPainter()
            painter.begin(image)

            # drawing robot trail
            # dreadfully inefficient as implemented
            if location not in D.trail: D.trail += [location]
            if DRAW_TRAIL and len(D.trail) > 1:
                painter.setPen(QtGui.QColor(255,0,0)) # red outline and fill
                painter.setBrush(QtGui.QColor(255,0,0))
                for p in range(1,len(D.trail)):
                    painter.drawLine(D.trail[p-1][0],D.trail[p-1][1],\
                        D.trail[p][0],D.trail[p][1])

            # drawing robot location
            painter.setPen(QtGui.QColor(0,0,0)) # black outline
            painter.setBrush(QtGui.QColor(255,255,255)) # white fill
            scale = 1.0 * 100.0**USE_CM # one pixel represents how many (c)m?
            painter.drawEllipse(\
                QtCore.QPoint(location[0],location[1]),6,6)

            painter.end()

        finally:
            self.imageLock.release()

        # We could  do more processing (eg OpenCV) here if we wanted to,
        # but for now lets just display the window.
        self.resize(image.width(),image.height())
        self.imageBox.setPixmap(image)

        # Status bar displays charge level only when updating previous level or
        # after a temporary message expires
        if self.statusBar().currentMessage() == "" or \
                "Charge" in self.statusBar().currentMessage():
            self.statusBar().showMessage("Charge: " + D.chargeLevel)

    @Slot()
    def SaveImage(self):
        """ receives click signal from saveButton """
        dialog = QtGui.QFileDialog(self)
        dialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
        dialog.setViewMode(QtGui.QFileDialog.Detail)
        fname, _ = dialog.getSaveFileName(self,\
            "Save image", "/home/robotics/Desktop/", "*.png")

        saving = self.imageBox.pixmap().toImage()
        
        if not saving.isNull() and fname != "":
            if fname[-4:] != ".png":
                fname += ".png"
            saving.save(fname, "png")
            self.setWindowTitle('RobotBox - ' + fname)
            self.statusBar().showMessage("Image saved to " + fname, 3000)
        elif saving.isNull():
            self.statusBar().showMessage("Failed to convert pixmap to image",\
                3000)

    @Slot()
    def LoadImage(self):
        """ receives click signal from loadButton """
        dialog = QtGui.QFileDialog(self)
        dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
        dialog.setViewMode(QtGui.QFileDialog.Detail)
        fname, _ = dialog.getOpenFileName(self,\
            "Load image", "/home/robotics/Desktop/", "*.png")
        
        if fname != "":
            D.loadedImage.load(fname)
            self.setWindowTitle('RobotBox - ' + fname)
            self.statusBar().showMessage("Image " + fname + " loaded", 3000)

    @Slot()
    def ClearImage(self):
        """ receives click signal from clearButton """
        D.loadedImage = QtGui.QImage()
        self.setWindowTitle('RobotBox')
        self.statusBar().showMessage("Cleared image", 3000)

    @Slot()
    def EraseTrail(self):
        """ receives click signal from eraseButton """
        if DRAW_TRAIL:
            D.trail = []
            self.statusBar().showMessage("Erased trail", 3000)
        else:
            self.statusBar().showMessage("What's a trail?", 3000)

    @Slot()
    def XReset(self):
        """ receives click signal from xResetButton """
        if math.ceil(D.x) != 0:
            D.xDiff = D.x
            D.trail = []

    @Slot()
    def YReset(self):
        """ receives click signal from yResetButton """
        if math.ceil(D.y) != 0:
            D.yDiff = D.y
            D.trail = []

    @Slot()
    def ThetaReset(self):
        """ receives click signal from thetaResetButton """
        if math.ceil(D.theta) != 0:
            D.thetaDiff = D.theta

    @Slot()
    def AllReset(self):
        """ receives click signal from allResetButton """
        self.XReset()
        self.YReset()
        self.ThetaReset()


def sensor_callback( data ):
    """ sensor_callback is called for each sensorPacket message
    """
    global D

    D.chargeLevel = str(int(round(data.chargeLevel * 100))) + '%'

    # data comes in as meters/radians
    D.x = data.x
    D.y = data.y
    D.theta = data.theta

    D.backLeftSensor = data.cliffLeftSignal
    D.frontLeftSensor = data.cliffFrontLeftSignal
    D.frontRightSensor = data.cliffFrontRightSignal
    D.backRightSensor = data.cliffRightSignal


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
