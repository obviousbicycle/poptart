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
SCALE = 1.0 # how many distance units for one pixel?


class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state

# visual-related globals
# for image loading
D.loadedImage = QtGui.QImage()
# toggle various things for display purposes
D.showRobot = True
D.makeTrail = 0
D.makeMCL = 0
# dependent on image size
D.minDensity = 1.0/90000

# location-related globals
# list of points where robot was during a GUI update
D.trail = []
# holds MCL particles and respective probabilities
D.particles = []
D.probabilities = []
# true if the robot has moved since last GUI update
D.recentMove = False

# odometry- and sensor-related globals
# for various data
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
# just because
D.chargeLevel = ""

# parameters changeable via slider
D.pointDensity = 0.01
D.noiseXY = 4.0
D.noiseTheta = 2.5


class RobotBox(QtGui.QMainWindow):
    def __init__(self):
        # Construct the parent class
        super(RobotBox, self).__init__()

        self.init_central()
        self.init_left()
        self.init_bottom()
        self.init_right()

    def init_central(self):
        """ Central Widget: imageBox
        Displays top-down map over the robot
        """
        # Setup our very basic GUI - a label which fills the whole
        # window and holds our image
        self.setWindowTitle('RobotBox')
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)
        
        # Holds the image frame received from the drone and later
        # processed by the GUI
        self.image = None
        self.imageLock = Lock()

        # Holds the status message to be displayed on the next
        # GUI update
        self.statusMessage = ''
        
        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.redraw_callback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

    def init_left(self):
        """Left Dock Widgets: positionGroup, actionGroup
        positionGroup displays position data from sensorPacket
        actionGroup shows image actions
        """
        leftBox = QtGui.QWidget(self)
        leftLayout = QtGui.QGridLayout()
        leftBox.setLayout(leftLayout)

        # positionGroup
        positionGroup = QtGui.QGroupBox("Position data")
        positionGroup.setAlignment(QtCore.Qt.AlignHCenter)
        # Position info + reset buttons
        # x data
        xLabel = QtGui.QLabel(self)
        xLabel.setText("x (" + USE_CM*"c" + "m): ")
        self.xValue = QtGui.QLabel(self)
        self.xValue.setText("")
        xResetButton = QtGui.QPushButton("Reset", self)
        xResetButton.clicked.connect(self.x_reset)
        # y data
        yLabel = QtGui.QLabel(self)
        yLabel.setText("y (" + USE_CM*"c" + "m): ")
        self.yValue = QtGui.QLabel(self)
        self.yValue.setText("")
        yResetButton = QtGui.QPushButton("Reset", self)
        yResetButton.clicked.connect(self.y_reset)
        # theta data
        thetaLabel = QtGui.QLabel(self)
        thetaLabel.setText("theta (deg): ")
        self.thetaValue = QtGui.QLabel(self)
        self.thetaValue.setText("")
        thetaResetButton = QtGui.QPushButton("Reset", self)
        thetaResetButton.clicked.connect(self.theta_reset)
        # reset all button
        allResetButton = QtGui.QPushButton("Reset all", self)
        allResetButton.clicked.connect(self.all_reset)
        # Assembling layout for positionGroup
        positionLayout = QtGui.QGridLayout()
        positionGroup.setLayout(positionLayout)
        positionLayout.setRowStretch(0, 1)
        # x
        positionLayout.addWidget(xLabel, 1, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.xValue, 1, 1)
        positionLayout.addWidget(xResetButton, 2, 1)
        positionLayout.setRowMinimumHeight(3, WIDGET_SPACING)
        # y
        positionLayout.addWidget(yLabel, 4, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.yValue, 4, 1)
        positionLayout.addWidget(yResetButton, 5, 1)
        positionLayout.setRowMinimumHeight(6, WIDGET_SPACING)
        # theta
        positionLayout.addWidget(thetaLabel, 7, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.thetaValue, 7, 1)
        positionLayout.addWidget(thetaResetButton, 8, 1)
        positionLayout.setRowMinimumHeight(9, 2*WIDGET_SPACING)
        # reset all
        positionLayout.addWidget(allResetButton, 10, 1)
        positionLayout.setRowStretch(11, 1)

        # actionGroup
        actionGroup = QtGui.QGroupBox("Image actions")
        actionGroup.setAlignment(QtCore.Qt.AlignHCenter)
        # Image action buttons
        # save
        self.saveButton = QtGui.QPushButton("&Save image", self)
        self.saveButton.clicked.connect(self.save_image)
        # load
        self.loadButton = QtGui.QPushButton("&Load map", self)
        self.loadButton.clicked.connect(self.load_image)
        # clear
        self.clearButton = QtGui.QPushButton("Clear image && map", self)
        self.clearButton.clicked.connect(self.clear_image)
        # Assembling layout for actionGroup
        actionLayout = QtGui.QGridLayout()
        actionGroup.setLayout(actionLayout)
        actionLayout.setRowStretch(0, 1)
        # save
        actionLayout.addWidget(self.saveButton, 1, 0, QtCore.Qt.AlignHCenter)
        # load
        actionLayout.addWidget(self.loadButton, 2, 0, QtCore.Qt.AlignHCenter)
        # clear
        actionLayout.addWidget(self.clearButton, 3, 0, QtCore.Qt.AlignHCenter)
        actionLayout.setRowStretch(4, 1)

        # Assembling top-level layout
        leftLayout.setRowStretch(0, 1)
        leftLayout.addWidget(positionGroup, 1, 0)
        leftLayout.addWidget(actionGroup, 2, 0)
        leftLayout.setRowStretch(3, 1)
        # Assembling dock
        leftDock = QtGui.QDockWidget("", self)
        leftDock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
        leftDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        leftDock.setTitleBarWidget(QtGui.QWidget(self))
        leftDock.setWidget(leftBox)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, leftDock)

    def init_bottom(self):
        """Bottom Dock Widget: lightBox
        Displays IR light sensor data from sensorPacket
        """
        # lightBox is just a generic widget holding a grid layout
        lightBox = QtGui.QWidget(self)

        readingsLabel = QtGui.QLabel(self)
        readingsLabel.setText("Readings")
        thresholdsLabel = QtGui.QLabel(self)
        thresholdsLabel.setText("Thresholds")

        # Light sensor info
        backLeftLabel = QtGui.QLabel(self)
        backLeftLabel.setText("Back left")
        self.backLeftValue = QtGui.QLabel(self)
        self.backLeftValue.setText("")

        frontLeftLabel = QtGui.QLabel(self)
        frontLeftLabel.setText("Front left")
        self.frontLeftValue = QtGui.QLabel(self)
        self.frontLeftValue.setText("")

        frontRightLabel = QtGui.QLabel(self)
        frontRightLabel.setText("Front right")
        self.frontRightValue = QtGui.QLabel(self)
        self.frontRightValue.setText("")

        backRightLabel = QtGui.QLabel(self)
        backRightLabel.setText("Back right")
        self.backRightValue = QtGui.QLabel(self)
        self.backRightValue.setText("")

        # Light threshold entry
        validator = QtGui.QIntValidator(0, 9999, self)

        self.backLeftField = QtGui.QLineEdit(str(D.backLeftThreshold))
        self.backLeftField.editingFinished.connect(self.back_left_set)
        self.backLeftField.setValidator(validator)

        self.frontLeftField = QtGui.QLineEdit(str(D.frontLeftThreshold))
        self.frontLeftField.editingFinished.connect(self.front_left_set)
        self.frontLeftField.setValidator(validator)

        self.frontRightField = QtGui.QLineEdit(str(D.frontRightThreshold))
        self.frontRightField.editingFinished.connect(self.front_right_set)
        self.frontRightField.setValidator(validator)

        self.backRightField = QtGui.QLineEdit(str(D.backRightThreshold))
        self.backRightField.editingFinished.connect(self.back_right_set)
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
        lightLayout.addWidget(self.backLeftValue, 2, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backLeftField, 3, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.setColumnMinimumWidth(4, WIDGET_SPACING)
        
        lightLayout.addWidget(frontLeftLabel, 1, 5, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftValue, 2, 5,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftField, 3, 5,
                              QtCore.Qt.AlignHCenter)
        lightLayout.setColumnMinimumWidth(6, WIDGET_SPACING)
        
        lightLayout.addWidget(frontRightLabel, 1, 7, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightValue, 2, 7,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightField, 3, 7,
                              QtCore.Qt.AlignHCenter)
        lightLayout.setColumnMinimumWidth(8, WIDGET_SPACING)
        
        lightLayout.addWidget(backRightLabel, 1, 9, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightValue, 2, 9,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightField, 3, 9,
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

    def init_right(self):
        """Right Dock Widget: displayGroup, parameterGroup
        displayGroup changes settings and stuff
        parameterGroup changes parameters and stuff
        """
        rightBox = QtGui.QWidget(self)
        rightLayout = QtGui.QGridLayout()
        rightBox.setLayout(rightLayout)

        # displayGroup
        displayGroup = QtGui.QGroupBox("Display settings")
        displayGroup.setAlignment(QtCore.Qt.AlignHCenter)
        # Robot marker
        robotLabel = QtGui.QLabel(self)
        robotLabel.setText("Robot marker")
        markerGroup = QtGui.QButtonGroup(self)
        self.showRobotButton = QtGui.QRadioButton("Show", self)
        self.showRobotButton.clicked.connect(self.show_robot)
        markerGroup.addButton(self.showRobotButton)
        self.hideRobotButton = QtGui.QRadioButton("Hide", self)
        self.hideRobotButton.clicked.connect(self.hide_robot)
        markerGroup.addButton(self.hideRobotButton)
        # Trails
        trailLabel = QtGui.QLabel(self)
        trailLabel.setText("Robot trail")
        trailGroup = QtGui.QButtonGroup(self)
        self.showTrailButton = QtGui.QRadioButton("Show", self)
        self.showTrailButton.clicked.connect(self.show_trail)
        trailGroup.addButton(self.showTrailButton)
        self.hideTrailButton = QtGui.QRadioButton("Hide", self)
        self.hideTrailButton.clicked.connect(self.hide_trail)
        trailGroup.addButton(self.hideTrailButton)
        self.noTrailButton = QtGui.QRadioButton("Off", self)
        self.noTrailButton.clicked.connect(self.no_trail)
        trailGroup.addButton(self.noTrailButton)
        self.eraseTrailButton = QtGui.QPushButton("Erase", self)
        self.eraseTrailButton.clicked.connect(self.erase_trail)
        # MCL
        MCLLabel = QtGui.QLabel(self)
        MCLLabel.setText("Monte Carlo")
        MCLGroup = QtGui.QButtonGroup(self)
        self.showMCLButton = QtGui.QRadioButton("Show", self)
        self.showMCLButton.clicked.connect(self.show_mcl)
        MCLGroup.addButton(self.showMCLButton)
        self.hide_mclButton = QtGui.QRadioButton("Hide", self)
        self.hide_mclButton.clicked.connect(self.hide_mcl)
        MCLGroup.addButton(self.hide_mclButton)
        self.noMCLButton = QtGui.QRadioButton("Off", self)
        self.noMCLButton.clicked.connect(self.no_mcl)
        MCLGroup.addButton(self.noMCLButton)
        self.erase_mclButton = QtGui.QPushButton("Erase", self)
        self.erase_mclButton.clicked.connect(self.erase_mcl)
        # Assembling layout for displayGroup
        displayLayout = QtGui.QGridLayout()
        displayGroup.setLayout(displayLayout)
        displayLayout.setColumnStretch(0, 1)
        displayLayout.setColumnMinimumWidth(2, WIDGET_SPACING)
        displayLayout.setColumnMinimumWidth(4, WIDGET_SPACING)
        displayLayout.setColumnStretch(6, 1)
        # first section: marker
        displayLayout.setRowStretch(0, 1)
        displayLayout.addWidget(robotLabel, 1, 1)
        displayLayout.addWidget(self.showRobotButton, 2, 1)
        displayLayout.addWidget(self.hideRobotButton, 2, 3)
        displayLayout.addWidget(QtGui.QWidget(self), 2, 5)
        displayLayout.setRowMinimumHeight(3, WIDGET_SPACING)
        # second section: trail
        displayLayout.addWidget(trailLabel, 4, 1)
        displayLayout.addWidget(self.showTrailButton, 5, 1)
        displayLayout.addWidget(self.hideTrailButton, 5, 2)
        displayLayout.addWidget(self.noTrailButton, 5, 3)
        displayLayout.addWidget(self.eraseTrailButton, 6, 2)
        displayLayout.setRowMinimumHeight(7, WIDGET_SPACING)
        # third section: mcl
        displayLayout.addWidget(MCLLabel, 8, 1)
        displayLayout.addWidget(self.showMCLButton, 9, 1)
        displayLayout.addWidget(self.hide_mclButton, 9, 2)
        displayLayout.addWidget(self.noMCLButton, 9, 3)
        displayLayout.addWidget(self.erase_mclButton, 10, 2)
        displayLayout.setRowStretch(11, 1)

        # parameterGroup
        parameterGroup = QtGui.QGroupBox("Parameters")
        parameterGroup.setAlignment(QtCore.Qt.AlignHCenter)
        # mcl point density
        self.densitySlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.densitySlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.densitySlider.setMinimum(D.minDensity)
        self.densitySlider.setMaximum(1.0)
        self.densitySlider.setTickInterval(0.001)
        self.densitySlider.setValue(D.pointDensity)
        self.densitySlider.valueChanged.connect(self.density_change)
        # Assembling layout for parameterGroup
        parameterLayout = QtGui.QGridLayout()
        parameterGroup.setLayout(parameterLayout)
        parameterLayout.setRowStretch(0, 1)
        parameterLayout.addWidget(self.densitySlider, 1, 0)
        parameterLayout.setRowStretch(20, 1)

        # assembling top-level layout
        rightLayout.setRowStretch(0, 1)
        rightLayout.addWidget(displayGroup, 1, 0)
        rightLayout.setRowStretch(2, 1)
        rightLayout.addWidget(parameterGroup, 3, 0)
        rightLayout.setRowStretch(4, 1)
        # assembling dock
        rightDock = QtGui.QDockWidget("", self)
        rightDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        rightDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        rightDock.setTitleBarWidget(QtGui.QWidget(self))
        rightDock.setWidget(rightBox)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, rightDock)

        # robot marker is shown and trail is not calculated upon start
        self.showRobotButton.setChecked(True)
        self.show_robot()
        self.noTrailButton.setChecked(True)
        self.no_trail()

    def redraw_callback(self):
        """ where the good stuff happens """
        global D

        self.imageLock.acquire()
        try:
            # updating infoBox values
            # if USE_CM = 1, values are converted here
            xDisplay = round(100.0**USE_CM * (D.x-D.xDiff), 2)
            yDisplay = round(100.0**USE_CM * (D.y-D.yDiff), 2)
            self.xValue.setText(str(xDisplay))
            self.yValue.setText(str(yDisplay))
            # convert theta to degrees and set bounds
            thetaDisplay = int(round(math.degrees(D.theta - D.thetaDiff)))
            while thetaDisplay > 360: thetaDisplay -= 360
            while thetaDisplay < 0: thetaDisplay += 360
            self.thetaValue.setText(str(thetaDisplay))
            # updating lightBox values
            self.backLeftValue.setText(str(D.backLeftSensor))
            self.frontLeftValue.setText(str(D.frontLeftSensor))
            self.frontRightValue.setText(str(D.frontRightSensor))
            self.backRightValue.setText(str(D.backRightSensor))

            # updating imageBox
            # right now, it just redraws the entire image every time
            # the GUI is updated...
            if D.loadedImage.isNull():
                # Default image is just a white square
                WIDTH = 300
                HEIGHT = 300
                blank = QtGui.QPixmap.fromImage(
                    QtGui.QImage(WIDTH, HEIGHT, QtGui.QImage.Format_RGB888))
                blank.fill(QtGui.QColor(255,255,255))
                image = blank
            else:
                WIDTH = D.loadedImage.width()
                HEIGHT = D.loadedImage.height()
                image = QtGui.QPixmap.fromImage(D.loadedImage)
            area = WIDTH*HEIGHT
            origin = [WIDTH/2, HEIGHT/2]
            # most graphics dealios consider top-left corner to be the
            # origin, so we need to adjust for that
            location = (origin[0] + SCALE*xDisplay,
                        origin[1] - SCALE*yDisplay,
                        D.theta)

            # drawing time
            painter = QtGui.QPainter()
            painter.begin(image)
            # drawing MCL particles
            # hold on to yer britches son cause this is one wild ride
            if D.makeTrail and D.makeMCL:
                numPoints = int(area*D.pointDensity)
                if not D.particles:
                    # initially, every point has an equal probability
                    # of being the robot's actual location
                    D.probabilities = [1.0/numPoints]*numPoints
                    # generate points with random x, y, theta
                    for n in range(numPoints):
                        p = [random.randint(0, WIDTH-1),
                             random.randint(0, HEIGHT-1),
                             random.randint(0,360)]
                        D.particles.append(p)
                elif len(D.trail) > 1:
                    # from the most recent motion data, calculate how
                    # far the particles must move
                    difference = [D.trail[-1][0]-D.trail[-2][0],
                                  D.trail[-1][1]-D.trail[-2][1],
                                  D.trail[-1][2]-D.trail[-2][2]]
                    if D.recentMove:
                        move = math.hypot(difference[0], difference[1])
                        oldGen = D.particles
                        oldProbs = D.probabilities
                        index = 0
                        for oldPt in oldGen:
                            # particles turn with robot
                            oldPt[2] += difference[2]
                            # apply robot's x and y change to particles
                            oldPt[0] += move*math.sin(math.radians(oldPt[2]))
                            oldPt[1] += move*math.cos(math.radians(oldPt[2]))
                            # particles that move off-screen are killed
                            if (oldPt[0] < 0 or oldPt[0] > WIDTH or
                                oldPt[1] < 0 or oldPt[1] > HEIGHT):
                                # killed particles unlikely to
                                # represent actual location, so we
                                # set their prob very low
                                oldProbs[index] = 0.0001
                            index += 1
                        # begin populating new generation with copies
                        # from old generation, based on the points'
                        # probabilities
                        newGen = []
                        newProbs = []
                        sumProb = math.fsum(oldProbs)
                        cumulativeProb = [math.fsum(oldProbs[i::-1]) for i in
                                          range(numPoints)]
                        counter = 0
                        for n in range(numPoints):
                            # step approach
                            newGen.append(oldGen[counter])
                            newProbs.append(oldProbs[counter]/sumProb)
                            while (
                              n*sumProb/numPoints > cumulativeProb[counter]):
                                counter += 1
                        # add some noise to each new point
                        D.particles = map(lambda p: [
                                            random.gauss(p[0],D.noiseXY),
                                            random.gauss(p[1],D.noiseXY),
                                            random.gauss(p[2],D.noiseTheta)
                                            ],
                                          newGen)
                        D.probabilities = newProbs
                # draw the particles now, if we want
                if D.makeMCL == 2:
                    pointRadius = 2
                    #mostRed = max(D.probabilities)
                    #mostPurple = min(D.probabilities)
                    index = 0
                    for p in D.probabilities:
                        #if mostRed == mostPurple:
                        #    hue = 150
                        #else:
                        #    hue = 300 * (p-mostPurple)/(mostRed-mostPurple)
                        color = QtGui.QColor.fromHsv(150,255,255)
                        painter.setPen(color)
                        painter.setBrush(color)
                        painter.drawEllipse(
                          QtCore.QPoint(
                            D.particles[index][0], D.particles[index][1]), 
                            pointRadius, pointRadius)
                        index += 1

            # drawing robot trail
            # dreadfully inefficient as implemented
            if D.makeTrail:
                if not D.trail or location != D.trail[-1]:
                    D.trail.append(location)
                    D.recentMove = True
                else:
                    D.recentMove = False
                if D.makeTrail == 2 and len(D.trail) > 1:
                    painter.setPen(QtGui.QColor(255,0,0))
                    painter.setBrush(QtGui.QColor(255,0,0))
                    for p in range(1,len(D.trail)):
                        painter.drawLine(D.trail[p-1][0], D.trail[p-1][1],
                                         D.trail[p][0], D.trail[p][1])

            # drawing robot location and pointer
            if D.showRobot:
                markerRadius = 6
                painter.setPen(QtGui.QColor(0,0,0)) # black outline
                painter.setBrush(QtGui.QColor(0,0,255)) # blue fill
                painter.drawEllipse(
                  QtCore.QPoint(location[0],location[1]),
                  markerRadius, markerRadius)

                painter.setPen(QtGui.QColor(0,0,255))
                distanceFromMarker = markerRadius + 3
                pointerLength = 8
                xDistance = distanceFromMarker*math.sin(
                  math.radians(thetaDisplay+90.0))
                yDistance = distanceFromMarker*math.cos(
                  math.radians(thetaDisplay+90.0))
                xLength = (xDistance + pointerLength*math.sin(
                  math.radians(thetaDisplay+90.0)))
                yLength = (yDistance + pointerLength*math.cos(
                  math.radians(thetaDisplay+90.0)))
                pointer = QtCore.QLineF(xDistance+location[0],
                                        yDistance+location[1],
                                        xLength+location[0],
                                        yLength+location[1])
                painter.drawLine(pointer)

            painter.end()

        finally:
            self.imageLock.release()

        # We could do more processing (eg OpenCV) here if we wanted
        # to, but for now let's just display the window.
        self.resize(image.width(),image.height())
        self.imageBox.setPixmap(image)

        # Status bar displays charge level only when updating previous
        # level or after a temporary message expires
        if (self.statusBar().currentMessage() == "" or 
                "Charge" in self.statusBar().currentMessage()):
            self.statusBar().showMessage("Charge: " + D.chargeLevel)

    # parameter changes
    @Slot()
    def density_change(self):
        D.pointDensity = self.densitySlider.value()

    # light threshold setters
    @Slot()
    def back_left_set(self):
        """ receives editingFinished signal from backLeftField """
        D.backLeftThreshold = int(self.backLeftField.text())

    @Slot()
    def front_left_set(self):
        """ receives editingFinished signal from frontLeftField """
        D.frontLeftThreshold = int(self.frontLeftField.text())

    @Slot()
    def front_right_set(self):
        """ receives editingFinished signal from frontRightField """
        D.frontRightThreshold = int(self.frontRightField.text())

    @Slot()
    def back_right_set(self):
        """ receives editingFinished signal from backRightField """
        D.backRightThreshold = int(self.backRightField.text())

    # image changers
    @Slot()
    def save_image(self):
        """ receives click signal from saveButton """
        dialog = QtGui.QFileDialog(self)
        dialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
        dialog.setViewMode(QtGui.QFileDialog.Detail)
        fname, _ = dialog.getSaveFileName(
          self, "Save image", "/home/robotics/Desktop/", "*.png")

        saving = self.imageBox.pixmap().toImage()
        if not saving.isNull() and fname:
            if fname[-4:] != ".png":
                fname += ".png"
            saving.save(fname, "png")
            self.setWindowTitle('RobotBox - ' + fname)
            self.statusBar().showMessage("Image saved to " + fname, 3000)
        elif saving.isNull():
            self.statusBar().showMessage(
              "Failed to convert pixmap to image", 3000)

    @Slot()
    def load_image(self):
        """ receives click signal from loadButton """
        dialog = QtGui.QFileDialog(self)
        dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
        dialog.setViewMode(QtGui.QFileDialog.Detail)
        fname, _ = dialog.getOpenFileName(
          self, "Load image", "/home/robotics/Desktop/", "*.png")
        
        if fname:
            D.loadedImage.load(fname)
            self.setWindowTitle('RobotBox - ' + fname)
            self.statusBar().showMessage("Image " + fname + " loaded", 3000)

    @Slot()
    def clear_image(self):
        """ receives click signal from clearButton """
        D.loadedImage = QtGui.QImage()
        self.setWindowTitle('RobotBox')
        self.statusBar().showMessage("Cleared image", 3000)

    # display setters
    @Slot()
    def show_robot(self):
        """ receives click signal from showRobotButton """
        D.showRobot = True

    @Slot()
    def hide_robot(self):
        """ receives click signal from hideRobotButton """
        D.showRobot = False

    @Slot()
    def show_trail(self):
        """ receives click signal from showTrailButton """
        self.eraseTrailButton.setEnabled(True)
        self.showMCLButton.setEnabled(True)
        self.hide_mclButton.setEnabled(True)
        D.makeTrail = 2

    @Slot()
    def hide_trail(self):
        """ receives click signal from hideTrailButton """
        self.eraseTrailButton.setEnabled(True)
        self.showMCLButton.setEnabled(True)
        self.hide_mclButton.setEnabled(True)
        D.makeTrail = 1

    @Slot()
    def no_trail(self):
        """ receives click signal from noTrailButton """
        self.erase_trail()
        self.no_mcl()
        self.eraseTrailButton.setEnabled(False)
        self.showMCLButton.setEnabled(False)
        self.hide_mclButton.setEnabled(False)
        self.noMCLButton.setChecked(True)
        self.statusBar().showMessage("Erased trail and MCL particles", 3000)
        D.makeTrail = 0

    @Slot()
    def erase_trail(self):
        """ receives click signal from eraseButton """
        D.trail = []
        self.statusBar().showMessage("Erased trail", 3000)

    @Slot()
    def show_mcl(self):
        """ receives click signal from showMCLButton """
        self.erase_mclButton.setEnabled(True)
        D.makeMCL = 2

    @Slot()
    def hide_mcl(self):
        """ receives click signal from hide_mclButton """
        self.erase_mclButton.setEnabled(True)
        D.makeMCL = 1

    @Slot()
    def no_mcl(self):
        """ receives click signal from noMCLButton """
        self.erase_mcl()
        self.erase_mclButton.setEnabled(False)
        D.makeMCL = 0

    @Slot()
    def erase_mcl(self):
        """ receives click signal from eraseButton """
        D.particles = []
        D.probabilities = []
        self.statusBar().showMessage("Erased MCL particles", 3000)

    # position data resetters
    @Slot()
    def x_reset(self):
        """ receives click signal from xResetButton """
        if math.ceil(D.x) != 0:
            D.xDiff = D.x
            D.trail = []
            D.particles = []

    @Slot()
    def y_reset(self):
        """ receives click signal from yResetButton """
        if math.ceil(D.y) != 0:
            D.yDiff = D.y
            D.trail = []
            D.particles = []

    @Slot()
    def theta_reset(self):
        """ receives click signal from thetaResetButton """
        if math.ceil(D.theta) != 0:
            D.thetaDiff = D.theta
            D.particles = []

    @Slot()
    def all_reset(self):
        """ receives click signal from allResetButton """
        self.x_reset()
        self.y_reset()
        self.theta_reset()


def sensor_callback( data ):
    """ sensor_callback is called for each sensorPacket message """
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
