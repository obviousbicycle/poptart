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
WIDGET_SPACING = 10 # pixels of space between widgets


class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state

D.chargeLevel = ""

# for image loading
D.loadedImage = QtGui.QImage()

# holds robot trail
D.trail = []

# holds MCL particles
D.particles = []

# true if the robot has moved since last GUI update
D.recentMove = False

# toggle various things
D.showRobot = True
D.makeTrail = 0
D.makeMCL = 0

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

# user-input thresholds
# feel free to change these default values to whatever you need
D.backLeftThreshold = 800
D.frontLeftThreshold = 700
D.frontRightThreshold = 1000
D.backRightThreshold = 1000


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
        positionBox = QtGui.QWidget(self.imageBox)

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
        positionBox.setLayout(positionLayout)

        positionLayout.setRowMinimumHeight(0, WIDGET_SPACING)
        positionLayout.addWidget(xLabel, 1, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.xValue, 1, 1)
        positionLayout.addWidget(xResetButton, 2, 1)
        positionLayout.setRowMinimumHeight(3, WIDGET_SPACING)
        positionLayout.addWidget(yLabel, 4, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.yValue, 4, 1)
        positionLayout.addWidget(yResetButton, 5, 1)
        positionLayout.setRowMinimumHeight(6, WIDGET_SPACING)
        positionLayout.addWidget(thetaLabel, 7, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.thetaValue, 7, 1)
        positionLayout.addWidget(thetaResetButton, 8, 1)
        positionLayout.setRowMinimumHeight(9, 2*WIDGET_SPACING)
        positionLayout.addWidget(allResetButton, 10, 1)
        positionLayout.setRowStretch(11, 1)

        positionTitle = QtGui.QLabel(self)
        positionTitle.setText("Position data")
        positionTitle.setAlignment(QtCore.Qt.AlignHCenter)

        positionDock = QtGui.QDockWidget("", self)
        positionDock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
        positionDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        positionDock.setTitleBarWidget(positionTitle)
        positionDock.setWidget(positionBox)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, positionDock)

    def initBottom(self):
        """
        Bottom Dock Widget: lightBox
        Displays IR light sensor data from sensorPacket
        """
        # lightBox is just a generic widget holding a grid layout
        lightBox = QtGui.QWidget(self)

        readingsLabel = QtGui.QLabel(self)
        readingsLabel.setText("Readings:")
        thresholdsLabel = QtGui.QLabel(self)
        thresholdsLabel.setText("Thresholds:")

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

        # Light threshold entry
        validator = QtGui.QIntValidator(0, 9999, self)

        self.backLeftField = QtGui.QLineEdit(str(D.backLeftThreshold))
        self.backLeftField.editingFinished.connect(self.backLeftSet)
        self.backLeftField.setValidator(validator)

        self.frontLeftField = QtGui.QLineEdit(str(D.frontLeftThreshold))
        self.frontLeftField.editingFinished.connect(self.frontLeftSet)
        self.frontLeftField.setValidator(validator)

        self.frontRightField = QtGui.QLineEdit(str(D.frontRightThreshold))
        self.frontRightField.editingFinished.connect(self.frontRightSet)
        self.frontRightField.setValidator(validator)

        self.backRightField = QtGui.QLineEdit(str(D.backRightThreshold))
        self.backRightField.editingFinished.connect(self.backRightSet)
        self.backRightField.setValidator(validator)

        # Putting together layout and dock widget
        lightLayout = QtGui.QGridLayout()
        lightBox.setLayout(lightLayout)

        lightLayout.setRowMinimumHeight(0, WIDGET_SPACING)
        lightLayout.setColumnStretch(0, 1)
        lightLayout.addWidget(readingsLabel, 2, 1, QtCore.Qt.AlignRight)
        lightLayout.addWidget(thresholdsLabel, 3, 1, QtCore.Qt.AlignRight)
        lightLayout.setColumnMinimumWidth(2, WIDGET_SPACING)
        
        lightLayout.addWidget(backLeftLabel, 1, 3, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backLeftValue, 2, 3,\
          QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backLeftField, 3, 3,\
          QtCore.Qt.AlignHCenter)
        lightLayout.setColumnMinimumWidth(4, WIDGET_SPACING)
        
        lightLayout.addWidget(frontLeftLabel, 1, 5, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftValue, 2, 5,\
          QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftField, 3, 5,\
          QtCore.Qt.AlignHCenter)
        lightLayout.setColumnMinimumWidth(6, WIDGET_SPACING)
        
        lightLayout.addWidget(frontRightLabel, 1, 7, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightValue, 2, 7,\
          QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightField, 3, 7,\
          QtCore.Qt.AlignHCenter)
        lightLayout.setColumnMinimumWidth(8, WIDGET_SPACING)
        
        lightLayout.addWidget(backRightLabel, 1, 9, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightValue, 2, 9,\
          QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightField, 3, 9,\
          QtCore.Qt.AlignHCenter)

        lightLayout.setColumnStretch(10, 1)
        lightLayout.setRowStretch(4, 1)

        lightTitle = QtGui.QLabel(self)
        lightTitle.setText("Light sensors")
        lightTitle.setAlignment(QtCore.Qt.AlignHCenter)

        lightDock = QtGui.QDockWidget("", self)
        lightDock.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea)
        lightDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        lightDock.setTitleBarWidget(lightTitle)
        lightDock.setWidget(lightBox)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, lightDock)

    def initRight(self):
        """
        Right Dock Widget: actionBox
        For changing settings and stuff
        """
        # actionBox is just a generic widget holding a grid layout
        actionBox = QtGui.QWidget(self)
        actionLayout = QtGui.QGridLayout(actionBox)
        actionBox.setLayout(actionLayout)

        # Image action buttons
        self.saveButton = QtGui.QPushButton("&Save image", self)
        self.saveButton.clicked.connect(self.SaveImage)

        self.loadButton = QtGui.QPushButton("&Load map", self)
        self.loadButton.clicked.connect(self.LoadImage)

        self.clearButton = QtGui.QPushButton("Clear image && map", self)
        self.clearButton.clicked.connect(self.ClearImage)

        # Display settings
        displayBox = QtGui.QWidget(actionBox)
        displayLayout = QtGui.QGridLayout(displayBox)
        displayBox.setLayout(displayLayout)
        displayLayout.setColumnStretch(0, 1)
        displayLayout.setColumnMinimumWidth(2, WIDGET_SPACING)
        displayLayout.setColumnMinimumWidth(4, WIDGET_SPACING)
        displayLayout.setColumnStretch(6, 1)

        # Robot marker
        robotLabel = QtGui.QLabel(self)
        robotLabel.setText("Robot marker:")

        markerGroup = QtGui.QButtonGroup(self)

        self.showRobotButton = QtGui.QRadioButton("Show", self)
        self.showRobotButton.clicked.connect(self.ShowRobot)
        markerGroup.addButton(self.showRobotButton)
        self.hideRobotButton = QtGui.QRadioButton("Hide", self)
        self.hideRobotButton.clicked.connect(self.HideRobot)
        markerGroup.addButton(self.hideRobotButton)

        displayLayout.setRowStretch(0, 1)
        displayLayout.addWidget(robotLabel, 1, 1)
        displayLayout.addWidget(self.showRobotButton, 2, 1)
        displayLayout.addWidget(self.hideRobotButton, 2, 3)
        displayLayout.addWidget(QtGui.QWidget(self), 2, 5)

        # Trails
        trailLabel = QtGui.QLabel(self)
        trailLabel.setText("Robot trail:")

        trailGroup = QtGui.QButtonGroup(self)

        self.showTrailButton = QtGui.QRadioButton("Show", self)
        self.showTrailButton.clicked.connect(self.ShowTrail)
        trailGroup.addButton(self.showTrailButton)
        self.hideTrailButton = QtGui.QRadioButton("Hide", self)
        self.hideTrailButton.clicked.connect(self.HideTrail)
        trailGroup.addButton(self.hideTrailButton)
        self.noTrailButton = QtGui.QRadioButton("Off", self)
        self.noTrailButton.clicked.connect(self.NoTrail)
        trailGroup.addButton(self.noTrailButton)
        self.eraseTrailButton = QtGui.QPushButton("Erase", self)
        self.eraseTrailButton.clicked.connect(self.EraseTrail)

        displayLayout.setRowMinimumHeight(3, WIDGET_SPACING)
        displayLayout.addWidget(trailLabel, 4, 1)
        displayLayout.addWidget(self.showTrailButton, 5, 1)
        displayLayout.addWidget(self.hideTrailButton, 5, 2)
        displayLayout.addWidget(self.noTrailButton, 5, 3)
        displayLayout.addWidget(self.eraseTrailButton, 6, 2)

        # MCL
        MCLLabel = QtGui.QLabel(self)
        MCLLabel.setText("Monte Carlo:")

        MCLGroup = QtGui.QButtonGroup(self)

        self.showMCLButton = QtGui.QRadioButton("Show", self)
        self.showMCLButton.clicked.connect(self.ShowMCL)
        MCLGroup.addButton(self.showMCLButton)
        self.hideMCLButton = QtGui.QRadioButton("Hide", self)
        self.hideMCLButton.clicked.connect(self.HideMCL)
        MCLGroup.addButton(self.hideMCLButton)
        self.noMCLButton = QtGui.QRadioButton("Off", self)
        self.noMCLButton.clicked.connect(self.NoMCL)
        MCLGroup.addButton(self.noMCLButton)
        self.eraseMCLButton = QtGui.QPushButton("Erase", self)
        self.eraseMCLButton.clicked.connect(self.EraseMCL)

        displayLayout.setRowMinimumHeight(7, WIDGET_SPACING)
        displayLayout.addWidget(MCLLabel, 8, 1)
        displayLayout.addWidget(self.showMCLButton, 9, 1)
        displayLayout.addWidget(self.hideMCLButton, 9, 2)
        displayLayout.addWidget(self.noMCLButton, 9, 3)
        displayLayout.addWidget(self.eraseMCLButton, 10, 2)
        displayLayout.setRowStretch(11, 1)

        # Putting together layout and dock widget
        actionLayout.addWidget(self.saveButton, 0, 0, QtCore.Qt.AlignHCenter)
        actionLayout.addWidget(self.loadButton, 1, 0, QtCore.Qt.AlignHCenter)
        actionLayout.addWidget(self.clearButton, 2, 0, QtCore.Qt.AlignHCenter)
        actionLayout.addWidget(displayBox, 3, 0)

        actionTitle = QtGui.QLabel(self)
        actionTitle.setText("Display settings")
        actionTitle.setAlignment(QtCore.Qt.AlignHCenter)

        actionDock = QtGui.QDockWidget("", self)
        actionDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        actionDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        actionDock.setTitleBarWidget(actionTitle)
        actionDock.setWidget(actionBox)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, actionDock)

        # robot marker is shown and trail is not calculated upon start
        self.showRobotButton.setChecked(True)
        self.ShowRobot()
        self.noTrailButton.setChecked(True)
        self.NoTrail()

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
            thetaDisplay = int(round(math.degrees(D.theta - D.thetaDiff)))
            while thetaDisplay > 360:
                thetaDisplay -= 360
            while thetaDisplay < 0:
                thetaDisplay += 360
            self.thetaValue.setText(str(thetaDisplay))

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

            area = WIDTH*HEIGHT
            scale = 1.0 # how many distance units for one pixel?
            origin = [WIDTH/2, HEIGHT/2]
            # most graphics dealios consider top-left corner to be the origin,
            # so we need to adjust for that
            location = (origin[0] + scale*xDisplay,\
              origin[1] - scale*yDisplay,\
              D.theta)

            painter = QtGui.QPainter()
            painter.begin(image)

            # drawing MCL particles
            # hold on to yer britches son cause this is one wild ride
            if D.makeTrail >= 1 and D.makeMCL >= 1:
                pDensity = int(area/100)
                if D.particles == []:
                    for n in range(pDensity):
                        # a point with random x, y, theta is generated
                        # initially, every point has an equal probability of
                        # being the robot's actual location
                        p = [random.randint(0, WIDTH-1),\
                          random.randint(0, HEIGHT-1),\
                          random.randint(0,360),\
                          1.0/pDensity]
                        D.particles.append(p)
                elif len(D.trail) > 1 and D.recentMove:
                    # from the most recent motion data, calculate how far the
                    # particles must move
                    difference = [D.trail[-1][0] - D.trail[-2][0],\
                      D.trail[-1][1] - D.trail[-2][1],\
                      D.trail[-1][2] - D.trail[-2][2]]
                    if D.recentMove:
                        move = math.hypot(difference[0], difference[1])
                        oldGen = D.particles
                        newGen = []
                        for oldPt in oldGen:
                            # if the robot turned, the particles must turn
                            oldPt[2] += difference[2]
                            # apply robot's x and y displacement to particles
                            xParticle = move * math.sin(math.radians(oldPt[2]))
                            yParticle = move * math.cos(math.radians(oldPt[2]))
                            oldPt[0] += xParticle
                            oldPt[1] += yParticle
                            # particles that move off-screen are killed
                            if oldPt[0] < 0 or oldPt[0] > WIDTH or\
                              oldPt[1] < 0 or oldPt[1] > HEIGHT:
                                # killed particles unlikely to represent actual
                                # location, so we set their prob very low
                                oldPt[3] = 0.0001
                        oldProbSum = reduce(lambda m,n: m+n,\
                          map(lambda p: p[3], oldGen))
                        # begin populating new generation with copies from
                        # old generation, based on the points' probabilities
                        for n in range(pDensity):
                            # dart-throwing approach
                            newProb = random.uniform(0, oldProbSum)
                            counter = 0
                            cumulativeProb = oldGen[0][3]
                            while newProb > cumulativeProb:
                                counter += 1
                                cumulativeProb += oldGen[counter][3]
                            newGen.append(oldGen[counter][:3]+[oldGen[counter][3]/oldProbSum])
                        # add some noise to each new point
                        # ... and recalculate probabilities while we're at it
                        noise = 4.0
                        noiseTheta = 2.5
                        newGen = map(lambda p: [random.gauss(p[0],noise),\
                                                random.gauss(p[1],noise),\
                                                random.gauss(p[2],noiseTheta),\
                                                p[3]],
                                                newGen)
                        # newGen is now ready
                        D.particles = newGen

                # draw the particles now, if we want
                if D.makeMCL == 2:
                    pRadius = 2
                    probs = map(lambda p: p[3], D.particles)
                    mostRed = max(probs)
                    mostPurple = min(probs)
                    for p in D.particles:
                        # a point is more red (hue = 0) if it is more probable
                        if mostRed == mostPurple: hue = 150
                        else: hue = 300 * (p[3]-mostPurple)/(mostRed-mostPurple)
                        color = QtGui.QColor.fromHsv(hue,255,255)
                        painter.setPen(color)
                        painter.setBrush(color)
                        painter.drawEllipse(\
                          QtCore.QPoint(p[0],p[1]),pRadius,pRadius)

            # drawing robot trail
            # dreadfully inefficient as implemented
            if D.makeTrail >= 1:
                if D.trail == [] or location != D.trail[-1]:
                    D.trail.append(location)
                    D.recentMove = True
                else: D.recentMove = False
                if D.makeTrail == 2 and len(D.trail) > 1:
                    painter.setPen(QtGui.QColor(255,0,0)) # red outline and fill
                    painter.setBrush(QtGui.QColor(255,0,0))
                    for p in range(1,len(D.trail)):
                        painter.drawLine(D.trail[p-1][0],D.trail[p-1][1],\
                          D.trail[p][0],D.trail[p][1])

            # drawing robot location
            if D.showRobot:
                painter.setPen(QtGui.QColor(0,0,0)) # black outline
                painter.setBrush(QtGui.QColor(0,0,255)) # blue fill
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
    def backLeftSet(self):
        """ receives editingFinished signal from backLeftField """
        D.backLeftThreshold = int(self.backLeftField.text())

    @Slot()
    def frontLeftSet(self):
        """ receives editingFinished signal from frontLeftField """
        D.frontLeftThreshold = int(self.frontLeftField.text())

    @Slot()
    def frontRightSet(self):
        """ receives editingFinished signal from frontRightField """
        D.frontRightThreshold = int(self.frontRightField.text())

    @Slot()
    def backRightSet(self):
        """ receives editingFinished signal from backRightField """
        D.backRightThreshold = int(self.backRightField.text())

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
    def ShowRobot(self):
        """ receives click signal from showRobotButton """
        D.showRobot = True

    @Slot()
    def HideRobot(self):
        """ receives click signal from hideRobotButton """
        D.showRobot = False

    @Slot()
    def ShowTrail(self):
        """ receives click signal from showTrailButton """
        self.eraseTrailButton.setEnabled(True)
        self.showMCLButton.setEnabled(True)
        self.hideMCLButton.setEnabled(True)
        D.makeTrail = 2

    @Slot()
    def HideTrail(self):
        """ receives click signal from hideTrailButton """
        self.eraseTrailButton.setEnabled(True)
        self.showMCLButton.setEnabled(True)
        self.hideMCLButton.setEnabled(True)
        D.makeTrail = 1

    @Slot()
    def NoTrail(self):
        """ receives click signal from noTrailButton """
        self.EraseTrail()
        self.NoMCL()
        self.eraseTrailButton.setEnabled(False)
        self.showMCLButton.setEnabled(False)
        self.hideMCLButton.setEnabled(False)
        self.noMCLButton.setChecked(True)
        self.statusBar().showMessage("Erased trail and MCL particles", 3000)
        D.makeTrail = 0

    @Slot()
    def EraseTrail(self):
        """ receives click signal from eraseButton """
        D.trail = []
        self.statusBar().showMessage("Erased trail", 3000)

    @Slot()
    def ShowMCL(self):
        """ receives click signal from showMCLButton """
        self.eraseMCLButton.setEnabled(True)
        D.makeMCL = 2

    @Slot()
    def HideMCL(self):
        """ receives click signal from hideMCLButton """
        self.eraseMCLButton.setEnabled(True)
        D.makeMCL = 1

    @Slot()
    def NoMCL(self):
        """ receives click signal from noMCLButton """
        self.EraseMCL()
        self.eraseMCLButton.setEnabled(False)
        D.makeMCL = 0

    @Slot()
    def EraseMCL(self):
        """ receives click signal from eraseButton """
        D.particles = []
        self.statusBar().showMessage("Erased MCL particles", 3000)

    @Slot()
    def XReset(self):
        """ receives click signal from xResetButton """
        if math.ceil(D.x) != 0:
            D.xDiff = D.x
            D.trail = []
            D.particles = []

    @Slot()
    def YReset(self):
        """ receives click signal from yResetButton """
        if math.ceil(D.y) != 0:
            D.yDiff = D.y
            D.trail = []
            D.particles = []

    @Slot()
    def ThetaReset(self):
        """ receives click signal from thetaResetButton """
        if math.ceil(D.theta) != 0:
            D.thetaDiff = D.theta
            D.particles = []

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
