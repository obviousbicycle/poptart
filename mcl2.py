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
USE_CM = True # set to True to use cm as main distance unit - and makes
              # for funny-looking but technically correct code
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
D.showAxes = False
D.showGrid = False
D.makeTrail = 0
D.makeMCL = 0
# robot marker offset in pixels
D.xOffset = 0
D.yOffset = 0
D.thetaOffset = 0

# location-related globals
# list of points where robot was during a GUI update
D.trail = []
# pixel coordinates of robot
# as far as I can tell, MCL only needs the current location and the
# previous location, not all of D.trail, and it doesn't make a lot of
# sense for MCL to depend on trails being on anyway
D.currentLocation = ()
D.previousLocation = ()
# true if the robot has moved since last GUI update
D.recentMove = False

# odometry- and sensor-related globals
# for various data
# in (centi)meters/degrees
D.x = 0.0
D.xPrevious = 0.0
D.y = 0.0
D.yPrevious = 0.0
D.theta = 0.0
# for "clearing" odometer
D.xDiff = 0.0
D.yDiff = 0.0
D.thetaDiff = 0.0
# for theta offset/reset purposes
D.pivot = ()
# IR light sensor data
D.sensors = [0,0,0,0]
# user-input thresholds
# feel free to change these default values to whatever you need
D.lowerThresholds = [75,80,130,135]
D.upperThresholds = [400,400,400,400]
# just because
D.chargeLevel = ""

# MCL-exclusive globals
# how many points in mcl
D.numParticles = int(D.width*D.height*0.01)
# mcl resampled points noise
D.xyNoise = 2.0
D.thetaNoise = 1.0 # currently unused
# use special particle coloring?
D.particleColoring = False
# list of white points on image representing white lines on floor
D.whiteLines = []
# list of black points on image representing black lines on floor
D.blackLines = []
# holds MCL particles and respective probabilities
D.particles = []
D.probabilities = []


# The whole GUI is in this class
class RobotBox(QtGui.QMainWindow):
    def __init__(self):
        # Construct the parent class
        super(RobotBox, self).__init__()

        # init each section of the main window
        self.init_menu()
        self.init_central()
        self.init_left()
        self.init_right()
        self.set_buttons()

    def init_menu(self):
        """Sets up main window's menu bar"""
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

    def init_central(self):
        """Central Widget: imageBox
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

    def init_left(self):
        """Left Dock Widgets: positionGroup, lightGroup
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
        lowerThresholdsLabel = QtGui.QLabel(self)
        lowerThresholdsLabel.setText("Lower\nthresholds")
        lowerThresholdsLabel.setAlignment(QtCore.Qt.AlignHCenter)
        upperThresholdsLabel = QtGui.QLabel(self)
        upperThresholdsLabel.setText("Upper\nthresholds")
        upperThresholdsLabel.setAlignment(QtCore.Qt.AlignHCenter)
        backLeftLabel = QtGui.QLabel(self)
        backLeftLabel.setText("Back\nleft")
        backLeftLabel.setAlignment(QtCore.Qt.AlignHCenter)
        frontLeftLabel = QtGui.QLabel(self)
        frontLeftLabel.setText("Front\nleft")
        frontLeftLabel.setAlignment(QtCore.Qt.AlignHCenter)
        frontRightLabel = QtGui.QLabel(self)
        frontRightLabel.setText("Front\nright")
        frontRightLabel.setAlignment(QtCore.Qt.AlignHCenter)
        backRightLabel = QtGui.QLabel(self)
        backRightLabel.setText("Back\nright")
        backRightLabel.setAlignment(QtCore.Qt.AlignHCenter)

        # Display values
        self.backLeftValue = QtGui.QLabel(self)
        self.backLeftValue.setText("")
        self.frontLeftValue = QtGui.QLabel(self)
        self.frontLeftValue.setText("")
        self.frontRightValue = QtGui.QLabel(self)
        self.frontRightValue.setText("")
        self.backRightValue = QtGui.QLabel(self)
        self.backRightValue.setText("")

        # Light threshold entry
        validator = QtGui.QIntValidator(0, 9999, self)
        self.backLeftLowerField = QtGui.QLineEdit(str(D.lowerThresholds[0]))
        self.backLeftLowerField.editingFinished.connect(
          self.back_left_lower_set)
        self.backLeftLowerField.setValidator(validator)
        self.frontLeftLowerField = QtGui.QLineEdit(str(D.lowerThresholds[1]))
        self.frontLeftLowerField.editingFinished.connect(
          self.front_left_lower_set)
        self.frontLeftLowerField.setValidator(validator)
        self.frontRightLowerField = QtGui.QLineEdit(str(D.lowerThresholds[2]))
        self.frontRightLowerField.editingFinished.connect(
          self.front_right_lower_set)
        self.frontRightLowerField.setValidator(validator)
        self.backRightLowerField = QtGui.QLineEdit(str(D.lowerThresholds[3]))
        self.backRightLowerField.editingFinished.connect(
          self.back_right_lower_set)
        self.backRightLowerField.setValidator(validator)
        self.backLeftUpperField = QtGui.QLineEdit(str(D.upperThresholds[0]))
        self.backLeftUpperField.editingFinished.connect(
          self.back_left_upper_set)
        self.backLeftUpperField.setValidator(validator)
        self.frontLeftUpperField = QtGui.QLineEdit(str(D.upperThresholds[1]))
        self.frontLeftUpperField.editingFinished.connect(
          self.front_left_upper_set)
        self.frontLeftUpperField.setValidator(validator)
        self.frontRightUpperField = QtGui.QLineEdit(str(D.upperThresholds[2]))
        self.frontRightUpperField.editingFinished.connect(
          self.front_right_upper_set)
        self.frontRightUpperField.setValidator(validator)
        self.backRightUpperField = QtGui.QLineEdit(str(D.upperThresholds[3]))
        self.backRightUpperField.editingFinished.connect(
          self.back_right_upper_set)
        self.backRightUpperField.setValidator(validator)

        # Assembling layout for lightGroup
        lightLayout = QtGui.QGridLayout()
        lightGroup.setLayout(lightLayout)
        lightLayout.addWidget(readingsLabel, 1, 2)
        lightLayout.addWidget(lowerThresholdsLabel, 1, 3)
        lightLayout.addWidget(upperThresholdsLabel, 1, 4)
        lightLayout.addWidget(backLeftLabel, 3, 1, QtCore.Qt.AlignRight)
        lightLayout.addWidget(self.backLeftValue, 3, 2,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backLeftLowerField, 3, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backLeftUpperField, 3, 4,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(frontLeftLabel, 5, 1, QtCore.Qt.AlignRight)
        lightLayout.addWidget(self.frontLeftValue, 5, 2,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftLowerField, 5, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontLeftUpperField, 5, 4,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(frontRightLabel, 7, 1, QtCore.Qt.AlignRight)
        lightLayout.addWidget(self.frontRightValue, 7, 2,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightLowerField, 7, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.frontRightUpperField, 7, 4,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(backRightLabel, 9, 1, QtCore.Qt.AlignRight)
        lightLayout.addWidget(self.backRightValue, 9, 2,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightLowerField, 9, 3,
                              QtCore.Qt.AlignHCenter)
        lightLayout.addWidget(self.backRightUpperField, 9, 4,
                              QtCore.Qt.AlignHCenter)
        # border stretch
        lightLayout.setRowStretch(0, 1)
        lightLayout.setRowStretch(10, 1)
        #lightLayout.setColumnStretch(4, 1)
        # spacing columns/rows
        #lightLayout.setColumnMinimumWidth(0, WIDGET_SPACING)
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
        """Right Dock Widget: displayGroup, parameterGroup, actionGroup
        displayGroup changes settings and stuff
        parameterGroup changes parameters and stuff
        actionGroup has image actions
        """
        rightTabs = QtGui.QTabWidget(self)

        ###
        # displayGroup
        ###
        displayGroup = QtGui.QWidget(self)

        # robot marker
        self.robotMarkerCheckbox = QtGui.QCheckBox("Robot marker", self)
        self.robotMarkerCheckbox.stateChanged.connect(
          self.robot_marker_change)

        # Axes
        self.axesCheckbox = QtGui.QCheckBox("Axes", self)
        self.axesCheckbox.stateChanged.connect(
          self.axes_change)

        # Grid
        self.gridCheckbox = QtGui.QCheckBox("Grid", self)
        self.gridCheckbox.stateChanged.connect(
          self.grid_change)

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
        self.hideMCLButton = QtGui.QRadioButton("Hide", self)
        self.hideMCLButton.clicked.connect(self.hide_mcl)
        self.noMCLButton = QtGui.QRadioButton("Off", self)
        self.noMCLButton.clicked.connect(self.no_mcl)
        self.eraseMCLButton = QtGui.QPushButton("Erase", self)
        self.eraseMCLButton.clicked.connect(self.erase_mcl)
        # limit radio button choices to MCL-related buttons
        MCLGroup = QtGui.QButtonGroup(self)
        MCLGroup.addButton(self.showMCLButton)
        MCLGroup.addButton(self.hideMCLButton)
        MCLGroup.addButton(self.noMCLButton)

        # Assembling layout for displayGroup
        displayLayout = QtGui.QGridLayout()
        displayGroup.setLayout(displayLayout)
        displayLayout.addWidget(self.robotMarkerCheckbox, 0, 1, 1, 2)
        displayLayout.addWidget(self.axesCheckbox, 1, 1)
        displayLayout.addWidget(self.gridCheckbox, 1, 2)
        displayLayout.addWidget(trailLabel, 3, 1, 1, 2)
        displayLayout.addWidget(self.showTrailButton, 4, 1)
        displayLayout.addWidget(self.hideTrailButton, 4, 2)
        displayLayout.addWidget(self.noTrailButton, 4, 3)
        displayLayout.addWidget(self.eraseTrailButton, 5, 1, 1, 3,
          QtCore.Qt.AlignLeft)
        displayLayout.addWidget(MCLLabel, 7, 1, 1, 2)
        displayLayout.addWidget(self.showMCLButton, 8, 1)
        displayLayout.addWidget(self.hideMCLButton, 8, 2)
        displayLayout.addWidget(self.noMCLButton, 8, 3)
        displayLayout.addWidget(self.eraseMCLButton, 9, 1, 1, 3,
          QtCore.Qt.AlignLeft)
        # blank rows/columns for spacing between widgets
        displayLayout.setRowMinimumHeight(2, WIDGET_SPACING)
        displayLayout.setRowMinimumHeight(6, WIDGET_SPACING)

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
        offsetLayout.addWidget(self.xOffsetField, 3, 1)
        offsetLayout.addWidget(self.xOffsetSlider, 4, 0, 1, 2)
        offsetLayout.addWidget(yOffsetLabel, 5, 0)
        offsetLayout.addWidget(self.yOffsetField, 5, 1)
        offsetLayout.addWidget(self.yOffsetSlider, 6, 0, 1, 2)
        offsetLayout.addWidget(thetaOffsetLabel, 7, 0)
        offsetLayout.addWidget(self.thetaOffsetField, 7, 1)
        offsetLayout.addWidget(self.thetaOffsetSlider, 8, 0, 1, 2)
        # spacing
        offsetLayout.setColumnStretch(0, 1)
        # border stretch
        offsetLayout.setRowStretch(0, 1)
        offsetLayout.setRowStretch(20, 1)
        
        ###
        # mclGroup
        ###
        mclGroup = QtGui.QWidget(self)

        # number of particles
        numParticlesLabel = QtGui.QLabel(self)
        numParticlesLabel.setText("# particles")
        numParticlesUsageLabel = QtGui.QLabel(self)
        numParticlesUsageLabel.setText("Erase current particles to update")
        # input field
        self.numParticlesField = QtGui.QLineEdit(str(D.numParticles))
        self.numParticlesField.editingFinished.connect(self.num_particles_set)
        numParticlesMax = D.width*D.height
        numParticlesValidator = QtGui.QIntValidator(1, numParticlesMax, self)
        self.numParticlesField.setValidator(numParticlesValidator)
        self.numParticlesField.setMaxLength(len(str(numParticlesMax)))

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
        self.thetaNoiseSlider.setValue(10*D.thetaNoise)
        self.thetaNoiseSlider.valueChanged.connect(self.theta_noise_change)

        # particle coloring
        self.particleColoringCheckbox = QtGui.QCheckBox(
          "Variable particle colors\n(may be slow)", self)
        self.particleColoringCheckbox.stateChanged.connect(
          self.particle_coloring_change)

        # mclGroup layout
        mclLayout = QtGui.QGridLayout()
        mclGroup.setLayout(mclLayout)
        mclLayout.addWidget(numParticlesLabel, 3, 0)
        mclLayout.addWidget(self.numParticlesField, 3, 1)
        mclLayout.addWidget(numParticlesUsageLabel, 4, 0, 1, 2)
        mclLayout.addWidget(xyNoiseLabel, 5, 0)
        mclLayout.addWidget(self.xyNoiseField, 5, 1)
        mclLayout.addWidget(self.xyNoiseSlider, 6, 0, 1, 2)
        mclLayout.addWidget(thetaNoiseLabel, 7, 0)
        mclLayout.addWidget(self.thetaNoiseField, 7, 1)
        mclLayout.addWidget(self.thetaNoiseSlider, 8, 0, 1, 2)
        mclLayout.addWidget(self.particleColoringCheckbox, 10, 0, 1, 2)
        # spacing
        mclLayout.setColumnStretch(0, 1)
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
        
    def set_buttons(self):
        """sets initial button states"""
        self.robotMarkerCheckbox.setChecked(True)
        self.robot_marker_change()
        self.noTrailButton.setChecked(True)
        self.no_trail()
        self.noMCLButton.setChecked(True)
        self.no_mcl()

    def redraw_callback(self):
        """where the good stuff happens """
        global D

        self.imageLock.acquire()
        try:
            # updating background image
            # it's always gray. features drawn later
            image = QtGui.QPixmap()
            image.convertFromImage(QtGui.QImage(
              D.width,D.height,QtGui.QImage.Format_RGB888))
            image.fill(QtGui.QColor.fromHsv(240,63,127))
            origin = (D.width/2, D.height/2)

            # updating light sensor readings
            self.backLeftValue.setText(str(D.sensors[0]))
            self.frontLeftValue.setText(str(D.sensors[1]))
            self.frontRightValue.setText(str(D.sensors[2]))
            self.backRightValue.setText(str(D.sensors[3]))

            # updating position
            # convert theta to degrees and set bounds
            # 0 degrees arbitrarily defined to be the horizontal and
            # theta increases in the counterclockwise direction
            thetaDisplay = math.degrees(D.theta-D.thetaDiff)+D.thetaOffset
            while thetaDisplay > 360: thetaDisplay -= 360
            while thetaDisplay < 0: thetaDisplay += 360
            self.thetaValue.setText(str(int(round(thetaDisplay))))

            xActual = D.x
            yActual = D.y
            if D.thetaDiff or D.thetaOffset:
                if D.pivot:
                    distance = math.hypot(D.x-D.pivot[0], D.y-D.pivot[1])
                    pivotAngle = math.atan2(D.y-D.pivot[1], D.x-D.pivot[0])
                    pivotAngle += 2*math.pi
                    xActual = D.pivot[0] + distance*math.cos(
                      pivotAngle+math.radians(D.thetaOffset))
                    yActual = D.pivot[1] + distance*math.sin(
                      pivotAngle+math.radians(D.thetaOffset))
                else:
                    D.pivot = (D.xPrevious,D.yPrevious)
            # if USE_CM = 1, values are converted here
            xDisplay = 100.0**USE_CM * (xActual-D.xDiff) + D.xOffset/SCALE
            yDisplay = 100.0**USE_CM * (yActual-D.yDiff) + D.yOffset/SCALE
            self.xValue.setText(str(round(xDisplay,2)))
            self.yValue.setText(str(round(yDisplay,2)))

            # updating locations
            D.previousLocation = D.currentLocation
            # most graphics dealios consider top-left corner to be the
            # origin, so we need to adjust for that
            D.currentLocation = (origin[0] + xDisplay*SCALE,
                                 origin[1] - yDisplay*SCALE,
                                 thetaDisplay)
            D.recentMove = not (D.previousLocation and
              D.previousLocation[0:1] == D.currentLocation[0:1])
            if D.makeTrail and D.recentMove:
                D.trail.append(D.currentLocation)

            # calculating MCL particles
            # hold on to yer britches son cause this is one wild ride
            if D.makeMCL:
                if not D.particles:
                    # initially, every point has an equal probability
                    # of being the robot's actual location
                    D.probabilities = [1.0/D.numParticles 
                                       for i in xrange(D.numParticles)]
                    # generate points with random x, y
                    for n in xrange(D.numParticles):
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
                        # motion update
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
                        # sensing update
                        # if IR readings exceed threshold, set
                        # probabilities corresponding to distance
                        # from image's white points
                        if (D.whiteLines and 
                                 D.sensors[1] >= D.upperThresholds[1]):
                            toWhiteLine = [
                                math.hypot(oldPt[0]-p[0],oldPt[1]-p[1])
                                for p in D.whiteLines
                                ]
                            oldProbs[index] *= 1/(1+math.sqrt(min(toWhiteLine)))
                        # elif (D.blackLines and
                        #          D.sensors[1] <= D.lowerThresholds[1]):
                        #     toBlackLine = [
                        #         math.hypot(oldPt[0]-p[0],oldPt[1]-p[1])
                        #         for p in D.blackLines
                        #         ]
                        #     oldProbs[index] *= 1/(1+min(toBlackLine))
                        index += 1
                    # begin populating new generation with copies
                    # from old generation, based on the points'
                    # probabilities
                    newGen = []
                    newProbs = []
                    sumProb = math.fsum(oldProbs)
                    cumulativeProb = [math.fsum(oldProbs[i::-1]) for i in
                                      xrange(len(D.particles))]
                    counter = 0
                    for n in xrange(len(D.particles)):
                        # step approach
                        # newGen.append(oldGen[counter])
                        # newProbs.append(oldProbs[counter]/sumProb)
                        # while (n*sumProb/len(D.particles) >
                        #         cumulativeProb[counter]):
                        #     counter += 1
                        newProb = random.uniform(0, sumProb)
                        counter = 0
                        cumulativeProb = oldProbs[0]
                        while newProb > cumulativeProb:
                            counter += 1
                            cumulativeProb += oldProbs[counter]
                        newGen.append(oldGen[counter])
                        newProbs.append(oldProbs[counter]/sumProb)
                # add some noise to each new point
                    D.particles = map(lambda p: [
                                        random.gauss(p[0],D.xyNoise),
                                        random.gauss(p[1],D.xyNoise)
                                        ],
                                      newGen)
                    D.probabilities = newProbs

            # time to draw
            painter = QtGui.QPainter()
            painter.begin(image)
            # drawing axes
            if D.showAxes:
                painter.setPen(QtGui.QColor.fromHsv(240,63,63))
                painter.setBrush(QtGui.QColor.fromHsv(240,63,63))
                painter.drawLine(D.width/2,0,D.width/2,D.height)
                painter.drawLine(0,D.height/2,D.width,D.height/2)
            # drawing grid
            if D.showGrid:
                spacing = 20
                painter.setPen(QtGui.QColor.fromHsv(240,63,63))
                painter.setBrush(QtGui.QColor.fromHsv(240,63,63))
                # grid is centered on origin
                for x in xrange((D.width/2)%spacing,D.width,spacing):
                    for y in xrange((D.height/2)%spacing,D.height,spacing):
                        painter.drawEllipse(x,y,1,1)
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
                        near = 10 # pixels
                        numClosePoints = len(filter(
                          lambda q: near >= abs(q[0]-p[0]) and near >=
                          abs(q[1]-p[1]), D.particles))
                        hue = 300*(1-numClosePoints/(D.numParticles*0.75))
                        if hue < 0.0: hue = 0
                        if hue > 300.0: hue = 300
                        color = QtGui.QColor.fromHsv(hue,255,200)
                        painter.setPen(color)
                        painter.setBrush(color)
                        painter.drawEllipse(QtCore.QPoint(p[0], p[1]), 
                          particleRadius, particleRadius)
                else:
                    painter.setPen(QtGui.QColor("darkGray"))
                    painter.setBrush(QtGui.QColor("darkGray"))
                    for p in D.particles:
                        painter.drawEllipse(QtCore.QPoint(p[0], p[1]), 
                          particleRadius, particleRadius)
            # drawing robot trail
            if D.makeTrail == 2 and len(D.trail) >= 2:
                painter.setPen(QtGui.QColor(255,0,0))
                painter.setBrush(QtGui.QColor(255,0,0))
                for p in xrange(1,len(D.trail)):
                    painter.drawLine(D.trail[p-1][0], D.trail[p-1][1],
                                     D.trail[p][0], D.trail[p][1])
            # drawing map features
            if D.whiteLines:
                painter.setPen(QtGui.QColor(255,255,255))
                painter.setBrush(QtGui.QColor(255,255,255))
                for p in D.whiteLines:
                    painter.drawPoint(p[0],p[1])
            if D.blackLines:
                painter.setPen(QtGui.QColor(0,0,0))
                painter.setBrush(QtGui.QColor(0,0,0))
                for p in D.blackLines:
                    painter.drawPoint(p[0],p[1])
            # drawing robot location and pointer
            if D.showRobot:
                markerRadius = 8
                painter.setPen(QtGui.QColor("black"))
                painter.setBrush(QtGui.QColor("lightGray"))
                painter.drawEllipse(
                  QtCore.QPoint(D.currentLocation[0],D.currentLocation[1]),
                  markerRadius, markerRadius)
                painter.setPen(QtGui.QColor(0,255,0))
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
        """receives valueChanged signal from xyNoiseSlider"""
        D.xyNoise = self.xyNoiseSlider.value()/10.0
        self.xyNoiseField.setText(str(D.xyNoise))

    @Slot()
    def xy_noise_set(self):
        """receives editingFinished signal from xyNoiseField"""
        D.xyNoise = float(self.xyNoiseField.text())
        self.xyNoiseSlider.blockSignals(True)
        self.xyNoiseSlider.setSliderPosition(10*D.xyNoise)
        self.xyNoiseSlider.blockSignals(False)

    @Slot()
    def theta_noise_change(self):
        """receives valueChanged signal from thetaNoiseSlider"""
        D.thetaNoise = self.thetaNoiseSlider.value()/10.0
        self.thetaNoiseField.setText(str(D.thetaNoise))

    @Slot()
    def theta_noise_set(self):
        """receives editingFinished signal from thetaNoiseField"""
        D.thetaNoise = float(self.thetaNoiseField.text())
        self.thetaNoiseSlider.blockSignals(True)
        self.thetaNoiseSlider.setSliderPosition(10*D.thetaNoise)
        self.thetaNoiseSlider.blockSignals(False)

    @Slot()
    def num_particles_set(self):
        """receives editingFinished signal from numParticlesField"""
        D.numParticles = int(self.numParticlesField.text())

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
        """receives valueChanged signal from xOffsetSlider"""
        D.xOffset = self.xOffsetSlider.value()
        self.xOffsetField.setText(str(D.xOffset))
        # the easy but lazy way:
        self.erase_trail() 
        self.erase_mcl()

    @Slot()
    def x_offset_set(self):
        """receives editingFinished signal from xOffsetField"""
        D.xOffset = int(self.xOffsetField.text())
        self.xOffsetSlider.blockSignals(True)
        self.xOffsetSlider.setSliderPosition(D.xOffset)
        self.xOffsetSlider.blockSignals(False)
        self.erase_trail()
        self.erase_mcl()

    @Slot()
    def y_offset_change(self):
        """receives valueChanged signal from yOffsetSlider"""
        D.yOffset = self.yOffsetSlider.value()
        self.yOffsetField.setText(str(D.yOffset))
        self.erase_trail()
        self.erase_mcl()

    @Slot()
    def y_offset_set(self):
        """receives editingFinished signal from yOffsetField"""
        D.yOffset = int(self.yOffsetField.text())
        self.yOffsetSlider.blockSignals(True)
        self.yOffsetSlider.setSliderPosition(D.yOffset)
        self.yOffsetSlider.blockSignals(False)
        self.erase_trail()
        self.erase_mcl()

    @Slot()
    def theta_offset_change(self):
        """receives valueChanged signal from thetaOffsetSlider"""
        D.thetaOffset = self.thetaOffsetSlider.value()
        self.thetaOffsetField.setText(str(D.thetaOffset))
        self.erase_trail()
        self.erase_mcl()
        D.pivot = (D.xPrevious,D.yPrevious)

    @Slot()
    def theta_offset_set(self):
        """receives editingFinished signal from thetaOffsetField"""
        D.thetaOffset = int(self.thetaOffsetField.text())
        self.thetaOffsetSlider.blockSignals(True)
        self.thetaOffsetSlider.setSliderPosition(D.thetaOffset)
        self.thetaOffsetSlider.blockSignals(False)
        self.erase_trail()
        self.erase_mcl()
        D.pivot = (D.xPrevious,D.yPrevious)

    # light threshold setters
    @Slot()
    def back_left_lower_set(self):
        """receives editingFinished signal from backLeftLowerField"""
        D.lowerThresholds[0] = int(self.backLeftLowerField.text())

    @Slot()
    def front_left_lower_set(self):
        """receives editingFinished signal from frontLeftLowerField"""
        D.lowerThresholds[1] = int(self.frontLeftLowerField.text())

    @Slot()
    def front_right_lower_set(self):
        """receives editingFinished signal from frontRightLowerField"""
        D.lowerThresholds[2] = int(self.frontRightLowerField.text())

    @Slot()
    def back_right_lower_set(self):
        """receives editingFinished signal from backRightLowerField"""
        D.lowerThresholds[3] = int(self.backRightLowerField.text())

    @Slot()
    def back_left_upper_set(self):
        """receives editingFinished signal from backLeftUpperField"""
        D.upperThresholds[0] = int(self.backLeftUpperField.text())

    @Slot()
    def front_left_upper_set(self):
        """receives editingFinished signal from frontLeftUpperField"""
        D.upperThresholds[1] = int(self.frontLeftUpperField.text())

    @Slot()
    def front_right_upper_set(self):
        """receives editingFinished signal from frontRightUpperField"""
        D.upperThresholds[2] = int(self.frontRightUpperField.text())

    @Slot()
    def back_right_upper_set(self):
        """receives editingFinished signal from backRightUpperField"""
        D.upperThresholds[3] = int(self.backRightUpperField.text())

    # image changers
    @Slot()
    def save_image(self):
        """receives click signal from saveButton"""
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
        """receives click signal from openButton"""
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
            D.whiteLines = []
            D.blackLines = []
            self.setWindowTitle('RobotBox - ' + fname)
            self.erase_trail()
            self.erase_mcl()
            # add black/white pixels to their lists
            for x in xrange(D.width):
                for y in xrange(D.height):
                    if (QtGui.QColor(toLoad.pixel(x,y)) ==
                            QtGui.QColor("white")):
                        D.whiteLines.append((x,y))
                    elif (QtGui.QColor(toLoad.pixel(x,y)) ==
                            QtGui.QColor("black")):
                        D.blackLines.append((x,y))
            self.statusBar().showMessage("Image " + fname + " opened", 3000)
        else:
            self.statusBar().showMessage("Failed to open image", 3000)

    @Slot()
    def clear_image(self):
        """receives click signal from clearButton"""
        D.width = DEFAULT_SIZE
        D.height = DEFAULT_SIZE
        D.whiteLines = []
        D.blackLines = []
        self.setWindowTitle('RobotBox')
        self.erase_trail()
        self.erase_mcl()
        self.statusBar().showMessage("Cleared image", 3000)

    @Slot()
    def robot_marker_change(self):
        """receives stateChanged signal from robotMarkerCheckbox"""
        D.showRobot = (True if
          self.robotMarkerCheckbox.isChecked() else False)

    @Slot()
    def axes_change(self):
        """receives stateChanged signal from axesCheckbox"""
        D.showAxes = (True if self.axesCheckbox.isChecked() else False)

    @Slot()
    def grid_change(self):
        """receives stateChanged signal from gridCheckbox"""
        D.showGrid = (True if self.gridCheckbox.isChecked() else False)

    @Slot()
    def show_trail(self):
        """receives click signal from showTrailButton"""
        self.eraseTrailButton.setEnabled(True)
        D.makeTrail = 2

    @Slot()
    def hide_trail(self):
        """receives click signal from hideTrailButton"""
        self.eraseTrailButton.setEnabled(True)
        D.makeTrail = 1

    @Slot()
    def no_trail(self):
        """receives click signal from noTrailButton"""
        self.erase_trail()
        self.eraseTrailButton.setEnabled(False)
        D.makeTrail = 0

    @Slot()
    def erase_trail(self):
        """receives click signal from eraseButton"""
        D.trail = []
        #self.statusBar().showMessage("Erased trail", 3000)

    @Slot()
    def show_mcl(self):
        """receives click signal from showMCLButton"""
        self.eraseMCLButton.setEnabled(True)
        D.makeMCL = 2

    @Slot()
    def hide_mcl(self):
        """receives click signal from hideMCLButton"""
        self.eraseMCLButton.setEnabled(True)
        D.makeMCL = 1

    @Slot()
    def no_mcl(self):
        """receives click signal from noMCLButton"""
        self.erase_mcl()
        self.eraseMCLButton.setEnabled(False)
        D.makeMCL = 0

    @Slot()
    def erase_mcl(self):
        """receives click signal from eraseButton"""
        D.particles = []
        D.probabilities = []
        #self.statusBar().showMessage("Erased MCL particles", 3000)

    # position data resetters
    # easy but lazy
    @Slot()
    def x_reset(self):
        """receives click signal from xResetButton"""
        D.xDiff = D.x
        self.erase_trail()
        self.erase_mcl()
        D.currentLocation = ()
        D.previousLocation = ()

    @Slot()
    def y_reset(self):
        """receives click signal from yResetButton"""
        D.yDiff = D.y
        self.erase_trail()
        self.erase_mcl()
        D.currentLocation = ()
        D.previousLocation = ()

    @Slot()
    def theta_reset(self):
        """receives click signal from thetaResetButton"""
        D.thetaDiff = D.theta
        self.erase_trail()
        self.erase_mcl()
        D.currentLocation = ()
        D.previousLocation = ()
        D.pivot = (D.xPrevious,D.yPrevious)

    @Slot()
    def all_reset(self):
        """receives click signal from allResetButton"""
        self.x_reset()
        self.y_reset()
        self.theta_reset()


def sensor_callback( data ):
    """sensor_callback is called for each sensorPacket message"""
    global D

    D.xPrevious = D.x
    D.yPrevious = D.y

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
