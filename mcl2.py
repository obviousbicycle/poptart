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

# We need to use resource locking to handle synchronization between GUI thread
# and ROS topic callbacks
from threading import Lock

# The GUI libraries
from PySide import QtCore, QtGui
from PySide.QtCore import Signal, Slot
import cv2

# General mathy stuff
import numpy as np
import random
import time
import math


# Some Constants
GUI_UPDATE_PERIOD = 20 # ms
USE_CM = 1 # set to 1 to use cm as main distance unit
SCALE = 1.0 # how many distance units for one pixel?
WIDGET_SPACING = 10 # pixels of space between widgets
DEFAULT_SIZE = 300 # side length of default image


class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state

# visual-related globals
# image dimensions
D.width = DEFAULT_SIZE
D.height = DEFAULT_SIZE
# toggle various things for display purposes
D.showRobot = True
D.makeTrail = 0
D.makeMCL = 0
# robot marker offset
D.xOffset = 0
D.yOffset = 0
D.thetaOffset = 0

# location-related globals
# list of points where robot was during a GUI update
D.trail = []
# as far as I can tell, MCL only needs the current location and the
# previous location, not all of D.trail, and it doesn't make a lot of
# sense for MCL to depend on trails being on anyway
D.currentLocation = ()
D.previousLocation = ()
# list of white points on image representing white lines on floor
D.lines = []
# holds MCL particles and respective probabilities
D.particles = []
D.probabilities = []
# true if the robot has moved since last GUI update
D.recentMove = False

# odometry- and sensor-related globals
# for various data
# in (centi)meters/degrees
D.x = 0.0
D.y = 0.0
D.theta = 0.0
# for "clearing" odometer
D.xDiff = 0.0
D.yDiff = 0.0
D.thetaDiff = 0.0
# IR light sensor data
D.sensors = [0]*4
# user-input thresholds
# feel free to change these default values to whatever you need
D.thresholds = [400,400,400,400]
# just because
D.chargeLevel = ""

# MCL-exclusive globals
# how many points in mcl (not actually changeable yet)
D.pointDensity = 0.01
# mcl resampled points noise
D.xyNoise = 4.0
D.thetaNoise = 2.5
# use special particle coloring?
D.particleColoring = False


# The whole GUI is in this class
class RobotBox(QtGui.QMainWindow):
    def __init__(self):
        # Construct the parent class
        super(RobotBox, self).__init__()

        # init each section of the main window
        self.init_central()
        self.init_menu()
        self.init_left()
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
        # currently unused whoops
        self.image = QtGui.QImage(D.width,D.height,QtGui.QImage.Format_RGB888)
        self.image.fill(QtGui.QColor(0,0,0))
        self.imageLock = Lock()

        # Holds the status message to be displayed on the next
        # GUI update
        self.statusMessage = ''
        
        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.redraw_callback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

    def init_menu(self):
        """ Sets up main window's menu bar """
        # image actions
        self.saveAction = QtGui.QAction("&Save image", self)
        self.saveAction.setShortcut(QtGui.QKeySequence("Ctrl+S"))
        self.saveAction.triggered.connect(self.save_image)
        self.openAction = QtGui.QAction("&Open map", self)
        self.openAction.triggered.connect(self.open_image)
        self.openAction.setShortcut(QtGui.QKeySequence("Ctrl+O"))
        self.clearAction = QtGui.QAction("Clear image && map", self)
        self.clearAction.triggered.connect(self.clear_image)

        self.fileMenu = self.menuBar().addMenu("Stuff")
        self.fileMenu.addAction(self.saveAction)
        self.fileMenu.addAction(self.openAction)
        self.fileMenu.addAction(self.clearAction)

    def init_left(self):
        """ Left Dock Widgets: positionGroup, lightGroup
        positionGroup displays position data from sensorPacket
        lightGroup displays IR light sensor data from sensorPacket
        """
        leftTabs = QtGui.QTabWidget(self)

        ###
        # positionGroup
        ###
        positionGroup = QtGui.QWidget(self)

        # x display
        xLabel = QtGui.QLabel(self)
        xLabel.setText("x (" + USE_CM*"c" + "m): ")
        self.xValue = QtGui.QLabel(self)
        self.xValue.setText("")
        xResetButton = QtGui.QPushButton("Reset", self)
        xResetButton.clicked.connect(self.x_reset)

        # y display
        yLabel = QtGui.QLabel(self)
        yLabel.setText("y (" + USE_CM*"c" + "m): ")
        self.yValue = QtGui.QLabel(self)
        self.yValue.setText("")
        yResetButton = QtGui.QPushButton("Reset", self)
        yResetButton.clicked.connect(self.y_reset)

        # theta display
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
        positionLayout.addWidget(xLabel, 1, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.xValue, 1, 1)
        positionLayout.addWidget(xResetButton, 2, 1)
        positionLayout.addWidget(yLabel, 4, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.yValue, 4, 1)
        positionLayout.addWidget(yResetButton, 5, 1)
        positionLayout.addWidget(thetaLabel, 7, 0, QtCore.Qt.AlignRight)
        positionLayout.addWidget(self.thetaValue, 7, 1)
        positionLayout.addWidget(thetaResetButton, 8, 1)
        positionLayout.addWidget(allResetButton, 10, 1)
        # increase stretch of border rows to keep the widgets close together
        positionLayout.setRowStretch(0, 1)
        positionLayout.setRowStretch(11, 1)
        # blank rows/columns for spacing between widgets
        positionLayout.setRowMinimumHeight(3, WIDGET_SPACING)
        positionLayout.setRowMinimumHeight(6, WIDGET_SPACING)
        positionLayout.setRowMinimumHeight(9, 2*WIDGET_SPACING)

        ###
        # lightGroup
        ###
        lightGroup = QtGui.QWidget(self)
        readingsLabel = QtGui.QLabel(self)
        readingsLabel.setText("Readings")
        thresholdsLabel = QtGui.QLabel(self)
        thresholdsLabel.setText("Thresholds")

        # Display values
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
        self.backLeftField = QtGui.QLineEdit(str(D.thresholds[0]))
        self.backLeftField.editingFinished.connect(self.back_left_set)
        self.backLeftField.setValidator(validator)
        self.frontLeftField = QtGui.QLineEdit(str(D.thresholds[1]))
        self.frontLeftField.editingFinished.connect(self.front_left_set)
        self.frontLeftField.setValidator(validator)
        self.frontRightField = QtGui.QLineEdit(str(D.thresholds[2]))
        self.frontRightField.editingFinished.connect(self.front_right_set)
        self.frontRightField.setValidator(validator)
        self.backRightField = QtGui.QLineEdit(str(D.thresholds[3]))
        self.backRightField.editingFinished.connect(self.back_right_set)
        self.backRightField.setValidator(validator)

        # Assembling layout for lightGroup
        lightLayout = QtGui.QGridLayout()
        lightGroup.setLayout(lightLayout)
        lightLayout.addWidget(readingsLabel, 1, 2, QtCore.Qt.AlignRight)
        lightLayout.addWidget(thresholdsLabel, 1, 3, QtCore.Qt.AlignRight)
        lightLayout.addWidget(backLeftLabel, 3, 1, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backLeftValue, 3, 2,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backLeftField, 3, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(frontLeftLabel, 5, 1, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftValue, 5, 2,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftField, 5, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(frontRightLabel, 7, 1, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightValue, 7, 2,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightField, 7, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(backRightLabel, 9, 1, QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightValue, 9, 2,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightField, 9, 3,
                              QtCore.Qt.AlignHCenter)
        # border stretch
        lightLayout.setRowStretch(0, 1)
        lightLayout.setRowStretch(10, 1)
        lightLayout.setColumnStretch(4, 1)
        # spacing columns/rows
        lightLayout.setColumnMinimumWidth(0, WIDGET_SPACING)
        lightLayout.setRowMinimumHeight(2, WIDGET_SPACING)
        lightLayout.setRowMinimumHeight(4, WIDGET_SPACING)
        lightLayout.setRowMinimumHeight(6, WIDGET_SPACING) 
        lightLayout.setRowMinimumHeight(8, WIDGET_SPACING)

        ###
        # Top-level
        ###
        leftTabs.addTab(positionGroup, "Position")
        leftTabs.addTab(lightGroup, "IR sensors")
        # Assembling dock
        leftDock = QtGui.QDockWidget("", self)
        leftDock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
        leftDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        leftDock.setTitleBarWidget(QtGui.QWidget(self))
        leftDock.setWidget(leftTabs)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, leftDock)

    def init_right(self):
        """ Right Dock Widget: displayGroup, parameterGroup, actionGroup
        displayGroup changes settings and stuff
        parameterGroup changes parameters and stuff
        actionGroup has image actions
        """
        rightTabs = QtGui.QTabWidget(self)

        ###
        # displayGroup
        ###
        displayGroup = QtGui.QWidget(self)

        # Robot marker
        robotLabel = QtGui.QLabel(self)
        robotLabel.setText("Robot marker")
        self.showRobotButton = QtGui.QRadioButton("Show", self)
        self.showRobotButton.clicked.connect(self.show_robot)
        self.hideRobotButton = QtGui.QRadioButton("Hide", self)
        self.hideRobotButton.clicked.connect(self.hide_robot)
        # limit radio button choices to marker-related buttons
        markerGroup = QtGui.QButtonGroup(self)
        markerGroup.addButton(self.showRobotButton)
        markerGroup.addButton(self.hideRobotButton)

        # Trails
        trailLabel = QtGui.QLabel(self)
        trailLabel.setText("Robot trail")
        self.showTrailButton = QtGui.QRadioButton("Show", self)
        self.showTrailButton.clicked.connect(self.show_trail)
        self.hideTrailButton = QtGui.QRadioButton("Hide", self)
        self.hideTrailButton.clicked.connect(self.hide_trail)
        self.noTrailButton = QtGui.QRadioButton("Off", self)
        self.noTrailButton.clicked.connect(self.no_trail)
        self.eraseTrailButton = QtGui.QPushButton("Erase", self)
        self.eraseTrailButton.clicked.connect(self.erase_trail)
        # limit radio button choices to trail-related buttons
        trailGroup = QtGui.QButtonGroup(self)
        trailGroup.addButton(self.showTrailButton)
        trailGroup.addButton(self.hideTrailButton)
        trailGroup.addButton(self.noTrailButton)

        # MCL
        MCLLabel = QtGui.QLabel(self)
        MCLLabel.setText("Monte Carlo")
        self.showMCLButton = QtGui.QRadioButton("Show", self)
        self.showMCLButton.clicked.connect(self.show_mcl)
        self.hide_mclButton = QtGui.QRadioButton("Hide", self)
        self.hide_mclButton.clicked.connect(self.hide_mcl)
        self.noMCLButton = QtGui.QRadioButton("Off", self)
        self.noMCLButton.clicked.connect(self.no_mcl)
        self.eraseMCLButton = QtGui.QPushButton("Erase", self)
        self.eraseMCLButton.clicked.connect(self.erase_mcl)
        # limit radio button choices to MCL-related buttons
        MCLGroup = QtGui.QButtonGroup(self)
        MCLGroup.addButton(self.showMCLButton)
        MCLGroup.addButton(self.hide_mclButton)
        MCLGroup.addButton(self.noMCLButton)

        # Assembling layout for displayGroup
        displayLayout = QtGui.QGridLayout()
        displayGroup.setLayout(displayLayout)
        displayLayout.addWidget(robotLabel, 1, 1)
        displayLayout.addWidget(self.showRobotButton, 2, 1)
        displayLayout.addWidget(self.hideRobotButton, 2, 3)
        displayLayout.addWidget(QtGui.QWidget(self), 2, 5)
        displayLayout.addWidget(trailLabel, 4, 1)
        displayLayout.addWidget(self.showTrailButton, 5, 1)
        displayLayout.addWidget(self.hideTrailButton, 5, 2)
        displayLayout.addWidget(self.noTrailButton, 5, 3)
        displayLayout.addWidget(self.eraseTrailButton, 6, 2)
        displayLayout.addWidget(MCLLabel, 8, 1)
        displayLayout.addWidget(self.showMCLButton, 9, 1)
        displayLayout.addWidget(self.hide_mclButton, 9, 2)
        displayLayout.addWidget(self.noMCLButton, 9, 3)
        displayLayout.addWidget(self.eraseMCLButton, 10, 2)
        # border stretch
        displayLayout.setColumnStretch(0, 1)
        displayLayout.setColumnStretch(6, 1)
        displayLayout.setRowStretch(0, 1)
        displayLayout.setRowStretch(11, 1)

        ###
        # offsetGroup
        ###

        # offsetGroup for moving robot marker around
        offsetGroup = QtGui.QWidget(self)

        # x
        xOffsetLabel = QtGui.QLabel(self)
        xOffsetLabel.setText("X")
        # x input field
        self.xOffsetField = QtGui.QLineEdit(str(D.xOffset))
        self.xOffsetField.editingFinished.connect(self.x_offset_set)
        xOffsetValidator = QtGui.QIntValidator(-D.width/2, D.width/2, self)
        self.xOffsetField.setValidator(xOffsetValidator)
        # x slider
        self.xOffsetSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.xOffsetSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.xOffsetSlider.setMinimum(-D.width/2)
        self.xOffsetSlider.setMaximum(D.width/2)
        self.xOffsetSlider.setTickInterval(D.width/20)
        self.xOffsetSlider.setValue(0)
        self.xOffsetSlider.valueChanged.connect(self.x_offset_change)

        # y
        yOffsetLabel = QtGui.QLabel(self)
        yOffsetLabel.setText("Y")
        # y input field
        self.yOffsetField = QtGui.QLineEdit(str(D.yOffset))
        self.yOffsetField.editingFinished.connect(self.y_offset_set)
        yOffsetValidator = QtGui.QIntValidator(-D.height/2, D.height/2, self)
        self.yOffsetField.setValidator(yOffsetValidator)
        # y slider
        self.yOffsetSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.yOffsetSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.yOffsetSlider.setMinimum(-D.height/2)
        self.yOffsetSlider.setMaximum(D.height/2)
        self.yOffsetSlider.setTickInterval(D.height/20)
        self.yOffsetSlider.setValue(0)
        self.yOffsetSlider.valueChanged.connect(self.y_offset_change)

        # theta
        thetaOffsetLabel = QtGui.QLabel(self)
        thetaOffsetLabel.setText("Theta")
        # theta input field
        self.thetaOffsetField = QtGui.QLineEdit(str(D.thetaOffset))
        self.thetaOffsetField.editingFinished.connect(self.theta_offset_set)
        thetaOffsetValidator = QtGui.QIntValidator(0, 359, self)
        self.thetaOffsetField.setValidator(thetaOffsetValidator)
        # theta slider
        self.thetaOffsetSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.thetaOffsetSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.thetaOffsetSlider.setMinimum(0)
        self.thetaOffsetSlider.setMaximum(359)
        self.thetaOffsetSlider.setTickInterval(15)
        self.thetaOffsetSlider.setValue(0)
        self.thetaOffsetSlider.valueChanged.connect(self.theta_offset_change)

        # offsetGroup layout
        offsetLayout = QtGui.QGridLayout()
        offsetGroup.setLayout(offsetLayout)
        offsetLayout.addWidget(xOffsetLabel, 3, 0)
        offsetLayout.addWidget(self.xOffsetField, 4, 0)
        offsetLayout.addWidget(self.xOffsetSlider, 4, 1)
        offsetLayout.addWidget(yOffsetLabel, 5, 0)
        offsetLayout.addWidget(self.yOffsetField, 6, 0)
        offsetLayout.addWidget(self.yOffsetSlider, 6, 1)
        offsetLayout.addWidget(thetaOffsetLabel, 7, 0)
        offsetLayout.addWidget(self.thetaOffsetField, 8, 0)
        offsetLayout.addWidget(self.thetaOffsetSlider, 8, 1)
        # border stretch
        offsetLayout.setRowStretch(0, 1)
        offsetLayout.setRowStretch(20, 1)
        
        ###
        # mclGroup
        ###
        mclGroup = QtGui.QWidget(self)

        # xy noise
        # apparently sliders only accept integer values??
        xyNoiseLabel = QtGui.QLabel(self)
        xyNoiseLabel.setText("XY noise")
        # xy input field
        self.xyNoiseField = QtGui.QLineEdit(str(D.xyNoise))
        self.xyNoiseField.editingFinished.connect(self.xy_noise_set)
        xyNoiseMax = 10.0
        xyNoiseValidator = QtGui.QDoubleValidator(0.0, xyNoiseMax, 1, self)
        self.xyNoiseField.setValidator(xyNoiseValidator)
        self.xyNoiseField.setMaxLength(len(str(xyNoiseMax)))
        # xy slider
        self.xyNoiseSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.xyNoiseSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.xyNoiseSlider.setMinimum(0)
        self.xyNoiseSlider.setMaximum(10*xyNoiseMax)
        self.xyNoiseSlider.setTickInterval(5)
        self.xyNoiseSlider.setValue(10*D.xyNoise)
        self.xyNoiseSlider.valueChanged.connect(self.xy_noise_change)

        # theta noise
        thetaNoiseLabel = QtGui.QLabel(self)
        thetaNoiseLabel.setText("Theta noise")
        # theta input field
        self.thetaNoiseField = QtGui.QLineEdit(str(D.thetaNoise))
        self.thetaNoiseField.editingFinished.connect(self.theta_noise_set)
        thetaNoiseMax = 5.0
        thetaNoiseValidator = QtGui.QDoubleValidator(
          0.0, thetaNoiseMax, 1, self)
        self.thetaNoiseField.setValidator(thetaNoiseValidator)
        self.thetaNoiseField.setMaxLength(len(str(thetaNoiseMax)))
        # theta slider
        self.thetaNoiseSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.thetaNoiseSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.thetaNoiseSlider.setMinimum(0)
        self.thetaNoiseSlider.setMaximum(10*thetaNoiseMax)
        self.thetaNoiseSlider.setTickInterval(5)
        self.thetaNoiseSlider.setSingleStep(5)
        self.thetaNoiseSlider.setValue(10*D.thetaNoise)
        self.thetaNoiseSlider.valueChanged.connect(self.theta_noise_change)

        # particle coloring
        self.particleColoringCheckbox = QtGui.QCheckBox(
          "Variable particle colors\n(may be expensive)", self)
        self.particleColoringCheckbox.stateChanged.connect(
          self.particle_coloring_change)

        # mclGroup layout
        mclLayout = QtGui.QGridLayout()
        mclGroup.setLayout(mclLayout)
        mclLayout.addWidget(xyNoiseLabel, 3, 0)
        mclLayout.addWidget(self.xyNoiseField, 4, 0)
        mclLayout.addWidget(self.xyNoiseSlider, 4, 1)
        mclLayout.addWidget(thetaNoiseLabel, 5, 0)
        mclLayout.addWidget(self.thetaNoiseField, 6, 0)
        mclLayout.addWidget(self.thetaNoiseSlider, 6, 1)
        mclLayout.addWidget(self.particleColoringCheckbox, 7, 0, 1, 2)
        # border stretch
        mclLayout.setRowStretch(0, 1)
        mclLayout.setRowStretch(20, 1)

        ###
        # Top-level
        ###
        rightTabs.addTab(displayGroup, "Draw")
        rightTabs.addTab(offsetGroup, "Offset")
        rightTabs.addTab(mclGroup, "MCL")
        # assembling dock
        rightDock = QtGui.QDockWidget("", self)
        rightDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        rightDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        rightDock.setTitleBarWidget(QtGui.QWidget(self))
        rightDock.setWidget(rightTabs)
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
            # updating position
            # if USE_CM = 1, values are converted here
            xDisplay = round(100.0**USE_CM * (D.x-D.xDiff), 2)+D.xOffset
            yDisplay = round(100.0**USE_CM * (D.y-D.yDiff), 2)+D.yOffset
            self.xValue.setText(str(xDisplay))
            self.yValue.setText(str(yDisplay))
            # convert theta to degrees and set bounds
            # 0 degrees arbitrarily defined to be the horizontal and
            # theta increases in the counterclockwise direction
            thetaDisplay = (int(round(math.degrees(D.theta - D.thetaDiff)))+
                            D.thetaOffset)
            while thetaDisplay > 360: thetaDisplay -= 360
            while thetaDisplay < 0: thetaDisplay += 360
            self.thetaValue.setText(str(thetaDisplay))

            # updating light sensor readings
            self.backLeftValue.setText(str(D.sensors[0]))
            self.frontLeftValue.setText(str(D.sensors[1]))
            self.frontRightValue.setText(str(D.sensors[2]))
            self.backRightValue.setText(str(D.sensors[3]))

            # updating background image
            # it's always black. features drawn later
            image = QtGui.QPixmap()
            image.convertFromImage(QtGui.QImage(
              D.width,D.height,QtGui.QImage.Format_RGB888))
            image.fill(QtGui.QColor(0,0,0))
            area = D.width*D.height
            origin = (D.width/2, D.height/2)

            # updating locations
            D.previousLocation = D.currentLocation
            # most graphics dealios consider top-left corner to be the
            # origin, so we need to adjust for that
            D.currentLocation = (origin[0] + SCALE*xDisplay,
                                 origin[1] - SCALE*yDisplay,
                                 D.theta)
            D.recentMove = (False if D.previousLocation == D.currentLocation
              or () else True)
            if D.makeTrail and D.recentMove:
                D.trail.append(D.currentLocation)

            # calculating MCL particles
            # hold on to yer britches son cause this is one wild ride
            if D.makeMCL:
                numPoints = int(area*D.pointDensity)
                if not D.particles:
                    # initially, every point has an equal probability
                    # of being the robot's actual location
                    D.probabilities = [1.0/numPoints 
                                       for i in range(numPoints)]
                    # generate points with random x, y, theta
                    for n in range(numPoints):
                        p = [random.randint(0, D.width-1),
                             random.randint(0, D.height-1)]
                        D.particles.append(p)
                elif D.recentMove:
                    # from the most recent motion data, calculate how
                    # far the particles must move
                    difference = (D.currentLocation[0]-D.previousLocation[0],
                                  D.currentLocation[1]-D.previousLocation[1])
                    oldGen = D.particles
                    oldProbs = D.probabilities
                    index = 0
                    for oldPt in oldGen:
                        # apply robot's x and y change to particles
                        oldPt[0] += difference[0]
                        oldPt[1] += difference[1]
                        # particles that move off-screen are killed
                        if (oldPt[0] < 0 or oldPt[0] > D.width or
                            oldPt[1] < 0 or oldPt[1] > D.height):
                            # killed particles unlikely to
                            # represent actual location, so we
                            # set their prob very low
                            oldProbs[index] = 0.000001
                        # if IR readings exceed threshold, set
                        # probabilities corresponding to distance
                        # from image's white points
                        if (D.lines and 
                                (D.sensors[0] >= D.thresholds[0] or
                                 D.sensors[1] >= D.thresholds[1] or
                                 D.sensors[2] >= D.thresholds[2] or
                                 D.sensors[3] >= D.thresholds[3])):
                            toLine = [
                                math.hypot(oldPt[0]-p[0],oldPt[1]-p[1])
                                for p in D.lines
                                ]
                            oldProbs[index] *= 1/(1+min(toLine))
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
                                        random.gauss(p[0],D.xyNoise),
                                        random.gauss(p[1],D.xyNoise)
                                        ],
                                      newGen)
                    D.probabilities = newProbs

            # time to draw
            # this ordering is very deliberate - particles drawn first,
            # then trails on top of that, then map features on top of
            # that, and finally robot marker always on top and visible
            painter = QtGui.QPainter()
            painter.begin(image)
            # drawing particles
            if D.makeMCL == 2:
                particleRadius = 2
                #distanceFromParticle = particleRadius+3
                #particlePointerLength = 4
                if D.particleColoring:
                    hue = 0
                    for p in D.particles:
                        # the color of one point is based on how many
                        # other points are close to it, "close" here
                        # being defined very arbitrarily
                        near = 5 # pixels
                        numClosePoints = len(filter(
                          lambda q: near >= abs(q[0]-p[0]) and near >=
                          abs(q[1]-p[1]), D.particles))
                        hue = 300*(1-numClosePoints/(numPoints*0.25))
                        if hue < 0.0: hue = 0
                        color = QtGui.QColor.fromHsv(hue,255,150)
                        painter.setPen(color)
                        painter.setBrush(color)
                        painter.drawEllipse(QtCore.QPoint(p[0], p[1]), 
                          particleRadius, particleRadius)
                else:
                    painter.setPen(QtGui.QColor("darkBlue"))
                    painter.setBrush(QtGui.QColor("darkBlue"))
                    for p in D.particles:
                        painter.drawEllipse(QtCore.QPoint(p[0], p[1]), 
                          particleRadius, particleRadius)
            # drawing robot trail
            if D.makeTrail == 2 and len(D.trail) >= 2:
                painter.setPen(QtGui.QColor(255,0,0))
                painter.setBrush(QtGui.QColor(255,0,0))
                for p in range(1,len(D.trail)):
                    painter.drawLine(D.trail[p-1][0], D.trail[p-1][1],
                                     D.trail[p][0], D.trail[p][1])
            # drawing map features
            if D.lines:
                painter.setPen(QtGui.QColor(255,255,255))
                painter.setBrush(QtGui.QColor(255,255,255))
                for p in D.lines:
                    painter.drawPoint(p[0],p[1])
            # drawing robot location and pointer
            if D.showRobot:
                markerRadius = 6
                painter.setPen(QtGui.QColor("cyan"))
                painter.setBrush(QtGui.QColor("cyan"))
                painter.drawEllipse(
                  QtCore.QPoint(D.currentLocation[0],D.currentLocation[1]),
                  markerRadius, markerRadius)

                painter.setPen(QtGui.QColor("cyan"))
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
                pointer = QtCore.QLineF(xDistance+D.currentLocation[0],
                                        yDistance+D.currentLocation[1],
                                        xLength+D.currentLocation[0],
                                        yLength+D.currentLocation[1])
                painter.drawLine(pointer)

            painter.end()
        finally:
            self.imageLock.release()

        # We could do more processing (eg OpenCV) here if we wanted
        # to, but for now let's just display the window.
        self.resize(D.width,D.height)
        self.imageBox.setPixmap(image)

        # Status bar displays charge level only when updating previous
        # level or after a temporary message expires
        if (not self.statusBar().currentMessage() or 
                "Charge" in self.statusBar().currentMessage()):
            self.statusBar().showMessage("Charge: " + D.chargeLevel)

    # mcl things
    @Slot()
    def xy_noise_change(self):
        """ receives valueChanged signal from xyNoiseSlider """
        D.xyNoise = self.xyNoiseSlider.value()/10.0
        self.xyNoiseField.setText(str(D.xyNoise))

    @Slot()
    def xy_noise_set(self):
        """ receives editingFinished signal from xyNoiseField """
        D.xyNoise = float(self.xyNoiseField.text())
        self.xyNoiseSlider.blockSignals(True)
        self.xyNoiseSlider.setSliderPosition(10*D.xyNoise)
        self.xyNoiseSlider.blockSignals(False)

    @Slot()
    def theta_noise_change(self):
        """ receives valueChanged signal from thetaNoiseSlider """
        D.thetaNoise = self.thetaNoiseSlider.value()/10.0
        self.thetaNoiseField.setText(str(D.thetaNoise))

    @Slot()
    def theta_noise_set(self):
        """ receives editingFinished signal from thetaNoiseField """
        D.thetaNoise = float(self.thetaNoiseField.text())
        self.thetaNoiseSlider.blockSignals(True)
        self.thetaNoiseSlider.setSliderPosition(10*D.thetaNoise)
        self.thetaNoiseSlider.blockSignals(False)

    @Slot()
    def particle_coloring_change(self):
        """receives stateChanged signal from
        particleColoringCheckbox
        """
        D.particleColoring = (True if
          self.particleColoringCheckbox.isChecked() else False)

    # offset changers
    @Slot()
    def x_offset_change(self):
        """ receives valueChanged signal from xOffsetSlider """
        D.xOffset = self.xOffsetSlider.value()
        self.xOffsetField.setText(str(D.xOffset))
        # the easy but lazy way:
        self.erase_trail() 
        self.erase_mcl()

    @Slot()
    def x_offset_set(self):
        """ receives editingFinished signal from xOffsetField """
        D.xOffset = int(self.xOffsetField.text())
        self.xOffsetSlider.blockSignals(True)
        self.xOffsetSlider.setSliderPosition(D.xOffset)
        self.xOffsetSlider.blockSignals(False)
        self.erase_trail()
        self.erase_mcl()

    @Slot()
    def y_offset_change(self):
        """ receives valueChanged signal from yOffsetSlider """
        D.yOffset = self.yOffsetSlider.value()
        self.yOffsetField.setText(str(D.yOffset))
        self.erase_trail()
        self.erase_mcl()

    @Slot()
    def y_offset_set(self):
        """ receives editingFinished signal from yOffsetField """
        D.yOffset = int(self.yOffsetField.text())
        self.yOffsetSlider.blockSignals(True)
        self.yOffsetSlider.setSliderPosition(D.yOffset)
        self.yOffsetSlider.blockSignals(False)
        self.erase_trail()
        self.erase_mcl()

    @Slot()
    def theta_offset_change(self):
        """ receives valueChanged signal from thetaOffsetSlider """
        D.thetaOffset = self.thetaOffsetSlider.value()
        self.thetaOffsetField.setText(str(D.thetaOffset))
        # I don't /think/ trails and MCL need to be erased

    @Slot()
    def theta_offset_set(self):
        """ receives editingFinished signal from thetaOffsetField """
        D.thetaOffset = int(self.thetaOffsetField.text())
        self.thetaOffsetSlider.blockSignals(True)
        self.thetaOffsetSlider.setSliderPosition(D.thetaOffset)
        self.thetaOffsetSlider.blockSignals(False)

    # light threshold setters
    @Slot()
    def back_left_set(self):
        """ receives editingFinished signal from backLeftField """
        D.thresholds[0] = int(self.backLeftField.text())

    @Slot()
    def front_left_set(self):
        """ receives editingFinished signal from frontLeftField """
        D.thresholds[1] = int(self.frontLeftField.text())

    @Slot()
    def front_right_set(self):
        """ receives editingFinished signal from frontRightField """
        D.thresholds[2] = int(self.frontRightField.text())

    @Slot()
    def back_right_set(self):
        """ receives editingFinished signal from backRightField """
        D.thresholds[3] = int(self.backRightField.text())

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
    def open_image(self):
        """ receives click signal from openButton """
        toLoad = QtGui.QImage()
        dialog = QtGui.QFileDialog(self)
        dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
        dialog.setViewMode(QtGui.QFileDialog.Detail)
        fname, _ = dialog.getOpenFileName(
          self, "Open image (must be black with white features)",
          "/home/robotics/Desktop/", "*.png")
        if toLoad.load(fname):
            D.width = toLoad.width()
            D.height = toLoad.height()
            D.lines = []
            self.setWindowTitle('RobotBox - ' + fname)
            self.erase_trail()
            self.erase_mcl()
            # add white pixels to D.lines
            for x in range(D.width):
                for y in range(D.height):
                    if (QtGui.QColor(toLoad.pixel(x,y)) ==
                            QtGui.QColor("white")):
                        D.lines.append((x,y))
            self.statusBar().showMessage("Image " + fname + " opened", 3000)
        else:
            self.statusBar().showMessage("Failed to open image", 3000)

    @Slot()
    def clear_image(self):
        """ receives click signal from clearButton """
        D.width = DEFAULT_SIZE
        D.height = DEFAULT_SIZE
        D.lines = []
        self.setWindowTitle('RobotBox')
        self.erase_trail()
        self.erase_mcl()
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
        D.makeTrail = 2

    @Slot()
    def hide_trail(self):
        """ receives click signal from hideTrailButton """
        self.eraseTrailButton.setEnabled(True)
        D.makeTrail = 1

    @Slot()
    def no_trail(self):
        """ receives click signal from noTrailButton """
        self.erase_trail()
        self.eraseTrailButton.setEnabled(False)
        D.makeTrail = 0

    @Slot()
    def erase_trail(self):
        """ receives click signal from eraseButton """
        D.trail = []
        #self.statusBar().showMessage("Erased trail", 3000)

    @Slot()
    def show_mcl(self):
        """ receives click signal from showMCLButton """
        self.eraseMCLButton.setEnabled(True)
        D.makeMCL = 2

    @Slot()
    def hide_mcl(self):
        """ receives click signal from hide_mclButton """
        self.eraseMCLButton.setEnabled(True)
        D.makeMCL = 1

    @Slot()
    def no_mcl(self):
        """ receives click signal from noMCLButton """
        self.erase_mcl()
        self.eraseMCLButton.setEnabled(False)
        D.makeMCL = 0

    @Slot()
    def erase_mcl(self):
        """ receives click signal from eraseButton """
        D.particles = []
        D.probabilities = []
        #self.statusBar().showMessage("Erased MCL particles", 3000)

    # position data resetters
    @Slot()
    def x_reset(self):
        """ receives click signal from xResetButton """
        D.xDiff = D.x
        D.trail = []
        D.particles = []

    @Slot()
    def y_reset(self):
        """ receives click signal from yResetButton """
        D.yDiff = D.y
        D.trail = []
        D.particles = []

    @Slot()
    def theta_reset(self):
        """ receives click signal from thetaResetButton """
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

    # data comes in as meters and radians
    D.x = data.x
    D.y = data.y
    D.theta = data.theta

    D.sensors = [data.cliffLeftSignal,
                 data.cliffFrontLeftSignal,
                 data.cliffFrontRightSignal,
                 data.cliffRightSignal]


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
