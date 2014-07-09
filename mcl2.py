#!/usr/bin/env python

# A window displaying the Create's movements based on its odometry

# General mathy stuff
import random
import time
import math
# We need to use resource locking to handle synchronization between GUI
# thread and ROS topic callbacks
from threading import Lock

# The GUI libraries
from PySide import QtCore, QtGui

# Import the ROS libraries, and load the manifest file which through
# <depend package=... /> will give us access to the project
# dependencies
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
# Import things relevant to subscribing directly to Create topics (?)
import irobot_mudd
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *


# Some Constants
GUI_UPDATE_PERIOD = 20 # ms
USE_CM = True # set to True to use cm as main distance unit
SCALE = 1.0 # how many distance units for one pixel?
WIDGET_SPACING = 10 # pixels of space between widgets
DEFAULT_SIZE = 300 # side length of default image


class Data: pass    # empty class for a generic data holder
D = Data()  # holds data directly received from robot's sensors


class Map(object): # TODO (?)

    """An image file interpreted as a map.

    Instance variables:
    black_lines -- black pixels representing black lines on floor
    white_lines -- white pixels representing white lines on floor

    Methods:
    light_of_point -- gets expected light value of given point
    """

    def __init__(self):
        self.black_lines = []
        self.white_lines = []

    def light_of_point(self, point):
        """Gets expected light value of given point"""
        if point in self.black_lines:
            return 0
        elif point in self.white_lines:
            return 1600
        else:
            return 200


class Sensor(object):

    """Represents IR light sensor on a (possibly hypothetical) robot.

    Instance variables:
    x -- the sensor's x-coordinate relative to the robot's center
    y -- the sensor's y-coordinate
    light -- the light value the sensor sees
    """

    def __init__(self, x, y):
        """Constructs one sensor."""
        self.x = x
        self.y = y
        self.light = -1


class Particle(object):

    """Represents a single MCL particle.

    Instance variables:
    x -- the x-coordinate of the particle on the image display
    y -- the y-coordinate
    theta -- the angle at which the particle would move forward,
        relative to the horizontal
    probability -- the probability that the robot's location
        matches this particle's location
    sensors -- an array of four sensors

    Class variable:
    sensor_angles -- an array of the angles of the four sensors
        relative to the direction the robot is facing

    Methods:
    displace -- a neat way to change x, y, theta
    update_sensor -- update sensors' (still local) coordinates
    get_sensor_global_position -- converts sensor's relative coordinates
        to global coordinates
    finish_resample -- the last thing a particle needs to do before it
        can really be part of a new generation
    """

    sensor_angles = [0, 0, 0, 0] # TODO

    def __init__(self, x, y, theta, probability, sensors=[]):
        """Construct the generic MCL particle"""
        self.x = x
        self.y = y
        self.theta = theta
        self.prob = probability
        self.sensors = sensors

    def displace(self, x=0, y=0, theta=0):
        """Moves the particle as specified by the arguments"""
        self.x += x
        self.y += y
        self.theta += theta

    def update_sensor_position(self, image_map=None):
        """Update sensor coordinates using particle's coordinates,
        then update sensor light values based on map
        """
        for s in self.sensors:
            distance = math.hypot(s.x**2 + s.y**2)
            angle = sensor_angles[self.sensors.index(s)] + self.theta
            s.x = math.cos(angle) * distance
            s.y = math.sin(angle) * distance
            if image_map is not None:
                s.light = image_map.light_of_point((s.x, s.y))

    def get_sensor_global_position(self):
        """Use particle's x, y, theta to convert its Sensor object
        coordinates to global coordinates
        """
        return [(s.x+self.x, s.y+self.y) for s in self.sensors]

    def finish_resample(self, xy_noise, theta_noise, psum):
        """Adds noise to x, y, theta, and divides prob by sum"""
        self.x = random.gauss(self.x, xy_noise)
        self.y = random.gauss(self.y, xy_noise)
        self.theta = random.gauss(self.theta, theta_noise)
        self.prob /= psum


class RobotBox(QtGui.QMainWindow):

    """The GUI

    Instance variables, not including widgets:
    width, height -- dimensions of display image
    make_trail -- toggle robot trail
    make_mcl -- toggle MCL
    x_offset, y_offset, theta_offset -- robot marker offset in pixels
    trail -- list of all past robot locations, from oldest to newest
    recent_move -- true if robot moved since last GUI update
    x, y, z -- robot coordinates as reported by robot
    xPrevious, yPrevious -- reported coordinates from previous GUI
        update
    xDiff, yDiff, thetaDiff -- for "clearing" odometer
    pivot -- for theta offset/reset purposes
    chargeLevel -- displays robot charge level
    sensors -- IR light sensor data
    lowerThresholds, upperThresholds -- user-input thresholds
    numParticles -- how many MCL particles
    particles -- list of Particle objects
    xyNoise, thetaNoise -- MCL resampled points noise, as sigma of
        Gaussian distribution
    imageMap -- image map of area
    image -- currently unused
    imageLock -- currently unused
    statusMessage -- the message in the window's bottom status bar
    redrawTimer -- redraws GUI with each cycle
    """

    def __init__(self):
        # Construct the parent class
        super(RobotBox, self).__init__()
        # Initialize data values related to visuals/drawing
        self.width = DEFAULT_SIZE
        self.height = DEFAULT_SIZE
        self.imageMap = Map()
        self.make_trail = 0
        self.make_mcl = 0
        self.x_offset = 0
        self.y_offset = 0
        self.theta_offset = 0
        # Initialize data values related to location
        self.trail = []
        self.recent_move = False
        # Initialize data values related to odometry/sensing
        self.xDiff = 0.0
        self.yDiff = 0.0
        self.thetaDiff = 0.0
        self.pivot = ()
        # first four are lower, last four are upper
        self.thresholds = [75, 80, 130, 135, 400, 400, 400, 400]
        # Initialize data values related to MCL
        self.numParticles = int(self.width * self.height * 0.01)
        self.particles = []
        self.particleRadius = 0
        self.xyNoise = 1.5
        self.thetaNoise = 1.0
        self.selected_particle = None
        # Initialize these so the program can run without robot
        D.x = 0.0
        D.y = 0.0
        D.theta = 0.0
        D.xPrevious = 0.0
        D.yPrevious = 0.0
        D.chargeLevel = ""
        D.sensors = [0, 0, 0, 0]
        # Initialize each section of the main window
        self.init_menu()
        self.init_central()
        self.init_left()
        self.init_right()

    def init_menu(self):
        """Sets up main window's menu bar"""
        self.saveAction = QtGui.QAction("&Save image", self)
        self.saveAction.setShortcut(QtGui.QKeySequence("Ctrl+S"))
        self.saveAction.triggered.connect(self.save_image)
        self.openAction = QtGui.QAction("&Open map", self)
        self.openAction.triggered.connect(self.open_image)
        self.openAction.setShortcut(QtGui.QKeySequence("Ctrl+O"))
        self.clearAction = QtGui.QAction("Clear image && map", self)
        self.clearAction.triggered.connect(self.clear_image)
        fileMenu = self.menuBar().addMenu("File")
        fileMenu.addAction(self.saveAction)
        fileMenu.addAction(self.openAction)
        fileMenu.addAction(self.clearAction)

    def init_central(self):
        """Central Widget: imageBox
        Displays top-down map over the robot
        """
        # Setup our very basic GUI - a label which fills the whole
        # window and holds our image
        self.setWindowTitle('RobotBox')
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)
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
        xLabel = QtGui.QLabel("X (" + USE_CM*"c" + "m): ", self)
        self.xValue = QtGui.QLabel(self)
        self.x_reset_button = QtGui.QPushButton("Reset", self)
        self.x_reset_button.clicked.connect(self.position_reset)

        # y display
        yLabel = QtGui.QLabel("Y (" + USE_CM*"c" + "m): ", self)
        self.yValue = QtGui.QLabel(self)
        self.y_reset_button = QtGui.QPushButton("Reset", self)
        self.y_reset_button.clicked.connect(self.position_reset)

        # theta display
        thetaLabel = QtGui.QLabel("Theta (deg): ", self)
        self.thetaValue = QtGui.QLabel(self)
        self.theta_reset_button = QtGui.QPushButton("Reset", self)
        self.theta_reset_button.clicked.connect(self.position_reset)

        # reset all button
        self.all_reset_button = QtGui.QPushButton("Reset all", self)
        self.all_reset_button.clicked.connect(self.position_reset)

        # Assembling layout for positionGroup
        positionLayout = QtGui.QGridLayout()
        positionGroup.setLayout(positionLayout)
        positionWidgetsToAdd = [
                      (xLabel, 1, 0, QtCore.Qt.AlignRight),
                 (self.xValue, 1, 1),
                (self.x_reset_button, 2, 1, QtCore.Qt.AlignLeft),
                      (yLabel, 4, 0, QtCore.Qt.AlignRight),
                 (self.yValue, 4, 1),
                (self.y_reset_button, 5, 1, QtCore.Qt.AlignLeft),
                  (thetaLabel, 7, 0, QtCore.Qt.AlignRight),
             (self.thetaValue, 7, 1),
            (self.theta_reset_button, 8, 1, QtCore.Qt.AlignLeft),
              (self.all_reset_button, 10, 1, QtCore.Qt.AlignLeft)
        ]
        for args in positionWidgetsToAdd: positionLayout.addWidget(*args)
        # increase stretch of border rows to keep the widgets close together
        positionLayout.setRowStretch(0, 1)
        positionLayout.setRowStretch(11, 1)
        # blank rows/columns for spacing between widgets
        positionLayout.setRowMinimumHeight(3, WIDGET_SPACING)
        positionLayout.setRowMinimumHeight(6, WIDGET_SPACING)
        positionLayout.setRowMinimumHeight(9, WIDGET_SPACING)

        ###
        # lightGroup
        ###
        lightGroup = QtGui.QWidget(self)
        readingsLabel = QtGui.QLabel("Readings", self)
        lowerThresholdsLabel = QtGui.QLabel("Lower\nthresholds", self)
        lowerThresholdsLabel.setAlignment(QtCore.Qt.AlignHCenter)
        upperThresholdsLabel = QtGui.QLabel("Upper\nthresholds", self)
        upperThresholdsLabel.setAlignment(QtCore.Qt.AlignHCenter)
        backLeftLabel = QtGui.QLabel("Back\nleft", self)
        backLeftLabel.setAlignment(QtCore.Qt.AlignHCenter)
        frontLeftLabel = QtGui.QLabel("Front\nleft", self)
        frontLeftLabel.setAlignment(QtCore.Qt.AlignHCenter)
        frontRightLabel = QtGui.QLabel("Front\nright", self)
        frontRightLabel.setAlignment(QtCore.Qt.AlignHCenter)
        backRightLabel = QtGui.QLabel("Back\nright", self)
        backRightLabel.setAlignment(QtCore.Qt.AlignHCenter)

        # Display values
        self.backLeftValue = QtGui.QLabel(self)
        self.frontLeftValue = QtGui.QLabel(self)
        self.frontRightValue = QtGui.QLabel(self)
        self.backRightValue = QtGui.QLabel(self)

        # Light threshold entry
        validator = QtGui.QIntValidator(0, 9999, self)
        self.backLeftLowerField = QtGui.QLineEdit(
          str(self.thresholds[0]))
        self.backLeftLowerField.editingFinished.connect(
          self.light_threshold_set)
        self.backLeftLowerField.setValidator(validator)
        self.frontLeftLowerField = QtGui.QLineEdit(
          str(self.thresholds[1]))
        self.frontLeftLowerField.editingFinished.connect(
          self.light_threshold_set)
        self.frontLeftLowerField.setValidator(validator)
        self.frontRightLowerField = QtGui.QLineEdit(
          str(self.thresholds[2]))
        self.frontRightLowerField.editingFinished.connect(
          self.light_threshold_set)
        self.frontRightLowerField.setValidator(validator)
        self.backRightLowerField = QtGui.QLineEdit(
          str(self.thresholds[3]))
        self.backRightLowerField.editingFinished.connect(
          self.light_threshold_set)
        self.backRightLowerField.setValidator(validator)
        self.backLeftUpperField = QtGui.QLineEdit(
          str(self.thresholds[4]))
        self.backLeftUpperField.editingFinished.connect(
          self.light_threshold_set)
        self.backLeftUpperField.setValidator(validator)
        self.frontLeftUpperField = QtGui.QLineEdit(
          str(self.thresholds[5]))
        self.frontLeftUpperField.editingFinished.connect(
          self.light_threshold_set)
        self.frontLeftUpperField.setValidator(validator)
        self.frontRightUpperField = QtGui.QLineEdit(
          str(self.thresholds[6]))
        self.frontRightUpperField.editingFinished.connect(
          self.light_threshold_set)
        self.frontRightUpperField.setValidator(validator)
        self.backRightUpperField = QtGui.QLineEdit(
          str(self.thresholds[7]))
        self.backRightUpperField.editingFinished.connect(
          self.light_threshold_set)
        self.backRightUpperField.setValidator(validator)

        # Assembling layout for lightGroup
        lightLayout = QtGui.QGridLayout()
        lightGroup.setLayout(lightLayout)
        lightWidgetsToAdd = [
                        (readingsLabel, 1, 2),
                 (lowerThresholdsLabel, 1, 3),
                 (upperThresholdsLabel, 1, 4),
                        (backLeftLabel, 3, 1, QtCore.Qt.AlignRight),
                   (self.backLeftValue, 3, 2, QtCore.Qt.AlignHCenter),
              (self.backLeftLowerField, 3, 3, QtCore.Qt.AlignHCenter),
              (self.backLeftUpperField, 3, 4, QtCore.Qt.AlignHCenter),
                       (frontLeftLabel, 5, 1, QtCore.Qt.AlignRight),
                  (self.frontLeftValue, 5, 2, QtCore.Qt.AlignHCenter),
             (self.frontLeftLowerField, 5, 3, QtCore.Qt.AlignHCenter),
             (self.frontLeftUpperField, 5, 4, QtCore.Qt.AlignHCenter),
                      (frontRightLabel, 7, 1, QtCore.Qt.AlignRight),
                 (self.frontRightValue, 7, 2, QtCore.Qt.AlignHCenter),
            (self.frontRightLowerField, 7, 3, QtCore.Qt.AlignHCenter),
            (self.frontRightUpperField, 7, 4, QtCore.Qt.AlignHCenter),
                       (backRightLabel, 9, 1, QtCore.Qt.AlignRight),
                  (self.backRightValue, 9, 2, QtCore.Qt.AlignHCenter),
             (self.backRightLowerField, 9, 3, QtCore.Qt.AlignHCenter),
             (self.backRightUpperField, 9, 4, QtCore.Qt.AlignHCenter)
        ]
        for args in lightWidgetsToAdd: lightLayout.addWidget(*args)
        # border stretch
        lightLayout.setRowStretch(0, 1)
        lightLayout.setRowStretch(10, 1)
        # spacing columns/rows
        lightLayout.setRowMinimumHeight(2, WIDGET_SPACING)
        lightLayout.setRowMinimumHeight(4, WIDGET_SPACING)
        lightLayout.setRowMinimumHeight(6, WIDGET_SPACING) 
        lightLayout.setRowMinimumHeight(8, WIDGET_SPACING)

        ###
        # particle_group
        ###
        particle_group = QtGui.QWidget(self)
        self.off_label = QtGui.QLabel("MCL is currently turned off.", self)
        self.off_label.setAlignment(QtCore.Qt.AlignCenter)
        self.hide_label = QtGui.QLabel("MCL is currently hidden.", self)
        self.hide_label.setAlignment(QtCore.Qt.AlignCenter)
        self.show_widget = QtGui.QWidget()
        show_label = QtGui.QLabel(
          "Double click a particle for more info.", self)
        show_label.setAlignment(QtCore.Qt.AlignHCenter)

        particle_info_group = QtGui.QGroupBox("Particle")
        particle_position_label = QtGui.QLabel("Position", self)
        self.particle_position_value = QtGui.QLabel(self)
        particle_theta_label = QtGui.QLabel("Theta")
        self.particle_theta_value = QtGui.QLabel(self)
        particle_weight_label = QtGui.QLabel("Weight", self)
        self.particle_weight_value = QtGui.QLabel(self)

        particle_info_layout = QtGui.QGridLayout()
        particle_info_group.setLayout(particle_info_layout)
        particle_info_widgets_to_add = [
                 (particle_position_label, 0, 0, QtCore.Qt.AlignRight),
            (self.particle_position_value, 0, 1),
                    (particle_theta_label, 1, 0, QtCore.Qt.AlignRight),
               (self.particle_theta_value, 1, 1),
                   (particle_weight_label, 3, 0, 1, 2, QtCore.Qt.AlignHCenter),
              (self.particle_weight_value, 5, 0, 1, 2, QtCore.Qt.AlignHCenter)
        ]
        for args in particle_info_widgets_to_add:
            particle_info_layout.addWidget(*args)
        particle_info_layout.setRowMinimumHeight(2, WIDGET_SPACING)
        particle_info_layout.setRowStretch(6, 1)

        particle_sensor_info_group = QtGui.QGroupBox("Sensors")
        particle_sensor_position_label = QtGui.QLabel("Position", self)
        particle_sensor_value_label = QtGui.QLabel("Value", self)
        particle_sensor_back_left_label = QtGui.QLabel("Back\nleft", self)
        particle_sensor_back_left_label.setAlignment(QtCore.Qt.AlignHCenter)
        particle_sensor_front_left_label = QtGui.QLabel("Front\nleft", self)
        particle_sensor_front_left_label.setAlignment(QtCore.Qt.AlignHCenter)
        particle_sensor_front_right_label = QtGui.QLabel("Front\nright", self)
        particle_sensor_front_right_label.setAlignment(QtCore.Qt.AlignHCenter)
        particle_sensor_back_right_label = QtGui.QLabel("Back\nright", self)
        particle_sensor_back_right_label.setAlignment(QtCore.Qt.AlignHCenter)
        self.particle_sensor_back_left_position = QtGui.QLabel(self)
        self.particle_sensor_front_left_position = QtGui.QLabel(self)
        self.particle_sensor_front_right_position = QtGui.QLabel(self)
        self.particle_sensor_back_right_position = QtGui.QLabel(self)
        self.particle_sensor_back_left_value = QtGui.QLabel(self)
        self.particle_sensor_front_left_value = QtGui.QLabel(self)
        self.particle_sensor_front_right_value = QtGui.QLabel(self)
        self.particle_sensor_back_right_value = QtGui.QLabel(self)

        particle_sensor_info_layout = QtGui.QGridLayout()
        particle_sensor_info_group.setLayout(particle_sensor_info_layout)
        particle_sensor_info_widgets_to_add = [
                       (particle_sensor_position_label, 0, 1),
                          (particle_sensor_value_label, 0, 2),
                      (particle_sensor_back_left_label, 1, 0),
                     (particle_sensor_front_left_label, 2, 0),
                    (particle_sensor_front_right_label, 3, 0),
                     (particle_sensor_back_right_label, 4, 0),
              (self.particle_sensor_back_left_position, 1, 1),
             (self.particle_sensor_front_left_position, 2, 1),
            (self.particle_sensor_front_right_position, 3, 1),
             (self.particle_sensor_back_right_position, 4, 1),
                 (self.particle_sensor_back_left_value, 1, 2),
                (self.particle_sensor_front_left_value, 2, 2),
               (self.particle_sensor_front_right_value, 3, 2),
                (self.particle_sensor_back_right_value, 4, 2)
        ]
        for args in particle_sensor_info_widgets_to_add:
            particle_sensor_info_layout.addWidget(*args)

        show_layout = QtGui.QGridLayout()
        self.show_widget.setLayout(show_layout)
        show_widgets_to_add = [
                            (show_label, 1, 0, 1, 2),
                   (particle_info_group, 2, 0),
            (particle_sensor_info_group, 2, 1)
        ]
        for args in show_widgets_to_add: show_layout.addWidget(*args)
        show_layout.setRowStretch(0, 1)
        show_layout.setRowStretch(3, 1)

        # Assembling layout for particle_group
        self.particle_layout = QtGui.QStackedLayout()
        particle_group.setLayout(self.particle_layout)
        particle_widgets_to_add = [
            self.off_label,
            self.hide_label,
            self.show_widget
        ]
        for arg in particle_widgets_to_add: self.particle_layout.addWidget(arg)
        self.particle_layout.setCurrentWidget(self.off_label)

        ###
        # Top-level
        ###
        leftTabs.addTab(positionGroup, "Position")
        leftTabs.addTab(lightGroup, "IR sensors")
        leftTabs.addTab(particle_group, "Particles")
        # Assembling dock
        self.leftDock = QtGui.QDockWidget("", self)
        self.leftDock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
        self.leftDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.leftDock.setTitleBarWidget(QtGui.QWidget(self))
        self.leftDock.setWidget(leftTabs)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.leftDock)

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

        # checkboxes
        self.robotMarkerCheckbox = QtGui.QCheckBox("Robot marker", self)
        self.robotMarkerCheckbox.setChecked(True)
        self.axesCheckbox = QtGui.QCheckBox("Axes", self)
        self.gridCheckbox = QtGui.QCheckBox("Grid", self)

        # Trails
        trailLabel = QtGui.QLabel("Robot trail", self)
        self.showTrailButton = QtGui.QRadioButton("Show", self)
        self.showTrailButton.clicked.connect(self.set_trail)
        self.hideTrailButton = QtGui.QRadioButton("Hide", self)
        self.hideTrailButton.clicked.connect(self.set_trail)
        self.noTrailButton = QtGui.QRadioButton("Off", self)
        self.noTrailButton.clicked.connect(self.set_trail)
        self.eraseTrailButton = QtGui.QPushButton("Erase", self)
        self.eraseTrailButton.clicked.connect(self.erase_trail)
        # limit radio button choices to trail-related buttons
        trailGroup = QtGui.QButtonGroup(self)
        trailGroup.addButton(self.showTrailButton)
        trailGroup.addButton(self.hideTrailButton)
        trailGroup.addButton(self.noTrailButton)

        # MCL
        MCLLabel = QtGui.QLabel("Monte Carlo", self)
        self.showMCLButton = QtGui.QRadioButton("Show", self)
        self.showMCLButton.clicked.connect(self.set_mcl)
        self.hideMCLButton = QtGui.QRadioButton("Hide", self)
        self.hideMCLButton.clicked.connect(self.set_mcl)
        self.noMCLButton = QtGui.QRadioButton("Off", self)
        self.noMCLButton.clicked.connect(self.set_mcl)
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
        displayWidgetsToAdd = [
            (self.robotMarkerCheckbox, 0, 1, 1, 2),
                   (self.axesCheckbox, 1, 1),
                   (self.gridCheckbox, 1, 2),
                          (trailLabel, 3, 1, 1, 2),
                (self.showTrailButton, 4, 1),
                (self.hideTrailButton, 4, 2),
                  (self.noTrailButton, 4, 3),
               (self.eraseTrailButton, 5, 1, 1, 3, QtCore.Qt.AlignLeft),
                            (MCLLabel, 7, 1, 1, 2),
                  (self.showMCLButton, 8, 1),
                  (self.hideMCLButton, 8, 2),
                    (self.noMCLButton, 8, 3),
                 (self.eraseMCLButton, 9, 1, 1, 3, QtCore.Qt.AlignLeft)
        ]
        for args in displayWidgetsToAdd: displayLayout.addWidget(*args)
        # blank rows/columns for spacing between widgets
        displayLayout.setRowMinimumHeight(2, WIDGET_SPACING)
        displayLayout.setRowMinimumHeight(6, WIDGET_SPACING)

        # set initial button states
        self.noTrailButton.setChecked(True)
        self.noMCLButton.setChecked(True)
        self.eraseTrailButton.setEnabled(False)
        self.eraseMCLButton.setEnabled(False)

        ###
        # offsetGroup
        ###

        # offsetGroup for moving robot marker around
        offsetGroup = QtGui.QWidget(self)

        # x
        x_offsetLabel = QtGui.QLabel("X", self)
        # x input field
        self.x_offset_field = QtGui.QLineEdit(str(self.x_offset))
        self.x_offset_field.editingFinished.connect(self.offset_field_change)
        x_offsetValidator = QtGui.QIntValidator(
          -self.width/2, self.width/2, self)
        self.x_offset_field.setValidator(x_offsetValidator)
        # x slider
        self.x_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.x_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.x_offset_slider.setMinimum(-self.width/2)
        self.x_offset_slider.setMaximum(self.width/2)
        self.x_offset_slider.setTickInterval(self.width/20)
        self.x_offset_slider.setValue(0)
        self.x_offset_slider.valueChanged.connect(self.offset_slider_change)

        # y
        y_offsetLabel = QtGui.QLabel("Y", self)
        # y input field
        self.y_offset_field = QtGui.QLineEdit(str(self.y_offset))
        self.y_offset_field.editingFinished.connect(self.offset_field_change)
        y_offsetValidator = QtGui.QIntValidator(
          -self.height / 2, self.height / 2, self)
        self.y_offset_field.setValidator(y_offsetValidator)
        # y slider
        self.y_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.y_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.y_offset_slider.setMinimum(-self.height / 2)
        self.y_offset_slider.setMaximum(self.height / 2)
        self.y_offset_slider.setTickInterval(self.height / 20)
        self.y_offset_slider.setValue(0)
        self.y_offset_slider.valueChanged.connect(self.offset_slider_change)

        # theta
        theta_offsetLabel = QtGui.QLabel("Theta", self)
        # theta input field
        self.theta_offset_field = QtGui.QLineEdit(str(self.theta_offset))
        self.theta_offset_field.editingFinished.connect(
          self.offset_field_change)
        theta_offsetValidator = QtGui.QIntValidator(0, 359, self)
        self.theta_offset_field.setValidator(theta_offsetValidator)
        # theta slider
        self.theta_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.theta_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.theta_offset_slider.setMinimum(0)
        self.theta_offset_slider.setMaximum(359)
        self.theta_offset_slider.setTickInterval(15)
        self.theta_offset_slider.setValue(0)
        self.theta_offset_slider.valueChanged.connect(
          self.offset_slider_change)

        # offsetGroup layout
        offsetLayout = QtGui.QGridLayout()
        offsetGroup.setLayout(offsetLayout)
        offsetWidgetsToAdd = [
                      (x_offsetLabel, 3, 0),
                 (self.x_offset_field, 3, 1),
                (self.x_offset_slider, 4, 0, 1, 2),
                      (y_offsetLabel, 5, 0),
                 (self.y_offset_field, 5, 1),
                (self.y_offset_slider, 6, 0, 1, 2),
                  (theta_offsetLabel, 7, 0),
             (self.theta_offset_field, 7, 1),
            (self.theta_offset_slider, 8, 0, 1, 2)
        ]
        for args in offsetWidgetsToAdd: offsetLayout.addWidget(*args)
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
        numParticlesLabel = QtGui.QLabel("# particles", self)
        # input field
        self.numParticlesField = QtGui.QLineEdit(str(self.numParticles))
        self.numParticlesField.editingFinished.connect(self.num_particles_set)
        numParticlesMax = self.width*self.height
        numParticlesValidator = QtGui.QIntValidator(1, numParticlesMax, self)
        self.numParticlesField.setValidator(numParticlesValidator)
        self.numParticlesField.setMaxLength(len(str(numParticlesMax)))

        # xy noise
        # apparently sliders only accept integer values??
        xyNoiseLabel = QtGui.QLabel("XY noise", self)
        # xy input field
        self.xyNoiseField = QtGui.QLineEdit(str(self.xyNoise))
        self.xyNoiseField.editingFinished.connect(self.noise_field_change)
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
        self.xyNoiseSlider.setValue(10*self.xyNoise)
        self.xyNoiseSlider.valueChanged.connect(self.noise_slider_change)

        # theta noise
        thetaNoiseLabel = QtGui.QLabel("Theta noise", self)
        # theta input field
        self.thetaNoiseField = QtGui.QLineEdit(str(self.thetaNoise))
        self.thetaNoiseField.editingFinished.connect(self.noise_field_change)
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
        self.thetaNoiseSlider.setValue(10*self.thetaNoise)
        self.thetaNoiseSlider.valueChanged.connect(self.noise_slider_change)

        # draw particles
        self.particleDetailCheckbox = QtGui.QCheckBox(
          "Detailed particles", self)
        self.particleColoringCheckbox = QtGui.QCheckBox(
          "Variable particle colors", self)

        # mclGroup layout
        mclLayout = QtGui.QGridLayout()
        mclGroup.setLayout(mclLayout)
        mclWidgetsToAdd = [
                        (numParticlesLabel, 3, 0),
                   (self.numParticlesField, 3, 1),
                             (xyNoiseLabel, 5, 0),
                        (self.xyNoiseField, 5, 1),
                       (self.xyNoiseSlider, 6, 0, 1, 2),
                          (thetaNoiseLabel, 7, 0),
                     (self.thetaNoiseField, 7, 1),
                    (self.thetaNoiseSlider, 8, 0, 1, 2),
              (self.particleDetailCheckbox, 9, 0, 1, 2),
            (self.particleColoringCheckbox, 10, 0, 1, 2)
        ]
        for args in mclWidgetsToAdd: mclLayout.addWidget(*args)
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

    def mouseDoubleClickEvent(self, event):
        """can double click on MCL particles"""
        if self.make_mcl == 2 and self.imageBox == self.childAt(event.pos()):
            relative_click_pos = (event.x()-self.leftDock.width()-6, event.y())
            for p in self.particles:
                distance = math.hypot(
                  relative_click_pos[0] - p.x, relative_click_pos[1] - p.y)
                if distance <= self.particleRadius:
                    self.selected_particle = p
                    break
            else: # obscure but legal for/else construct
                self.selected_particle = None

    def redraw_callback(self):
        """updates the GUI"""
        global D
        # updating background image
        # it's always gray. features drawn later
        image = QtGui.QPixmap()
        image.convertFromImage(QtGui.QImage(
          self.width,self.height,QtGui.QImage.Format_RGB888))
        image.fill(QtGui.QColor.fromHsv(240, 63, 127))

        # updating light sensor readings
        self.backLeftValue.setText(str(D.sensors[0]))
        self.frontLeftValue.setText(str(D.sensors[1]))
        self.frontRightValue.setText(str(D.sensors[2]))
        self.backRightValue.setText(str(D.sensors[3]))

        # updating position
        # convert theta to degrees and set bounds
        # 0 degrees arbitrarily defined to be the horizontal and
        # theta increases in the counterclockwise direction
        thetaDisplay = math.degrees(D.theta-self.thetaDiff) + self.theta_offset
        while thetaDisplay > 360: thetaDisplay -= 360
        while thetaDisplay < 0: thetaDisplay += 360
        self.thetaValue.setText(str(int(round(thetaDisplay))))
        xActual = D.x
        yActual = D.y
        if self.thetaDiff or self.theta_offset:
            if self.pivot:
                distance = math.hypot(D.x-self.pivot[0], D.y-self.pivot[1])
                pivotAngle = math.atan2(D.y-self.pivot[1], D.x-self.pivot[0])
                pivotAngle += 2*math.pi
                xActual = self.pivot[0] + distance*math.cos(
                  pivotAngle+math.radians(self.theta_offset))
                yActual = self.pivot[1] + distance*math.sin(
                  pivotAngle+math.radians(self.theta_offset))
            else:
                self.pivot = (D.xPrevious,D.yPrevious)
        # if USE_CM = 1, values are converted here
        xDisplay = 100.0**USE_CM * (xActual-self.xDiff) + self.x_offset/SCALE
        yDisplay = 100.0**USE_CM * (yActual-self.yDiff) + self.y_offset/SCALE
        self.xValue.setText(str(round(xDisplay, 2)))
        self.yValue.setText(str(round(yDisplay, 2)))

        # updating locations
        # most graphics dealios consider top-left corner to be the
        # origin, so we need to adjust for that
        origin = (self.width / 2, self.height / 2)
        self.trail.append((origin[0] + xDisplay*SCALE,
                        origin[1] - yDisplay*SCALE,
                        thetaDisplay))
        self.recent_move = (len(self.trail) >= 2 and 
          self.trail[-2] != self.trail[-1])
        if not self.make_trail and self.recent_move:
            del self.trail[0]

        if self.make_mcl:
           self.mcl_update()
        # if self.make_mcl:
        #     whiteExpected = 400
        #     blackExpected = 150
        #     neutralExpected = 200
        #     if not self.particles:
        #         # generate points with random x, y, and an
        #         # arbitrary expected light value
        #         # initially, every point has an equal probability
        #         # of being the robot's actual location
        #         for n in xrange(self.numParticles):
        #             p = [random.randint(0, self.width-1),
        #                  random.randint(0, self.height-1),
        #                  neutralExpected,
        #                  1.0 / self.numParticles]
        #             self.particles.append(p)
        #     elif self.recent_move:
        #         # from the most recent motion data, calculate how
        #         # far the particles must move
        #         difference = (self.trail[-1][0] - self.trail[-2][0],
        #                       self.trail[-1][1] - self.trail[-2][1])
        #         oldGen = self.particles
        #         for oldPt in oldGen:
        #             # motion update
        #             # apply robot's x and y change to particles
        #             oldPt[0] += difference[0]
        #             oldPt[1] += difference[1]
        #             # particles that move off-screen are killed
        #             if (oldPt[0] < 0 or oldPt[0] > self.width or
        #                 oldPt[1] < 0 or oldPt[1] > self.height):
        #                 # killed particles unlikely to
        #                 # represent actual location, so we
        #                 # set their prob very low
        #                 oldPt[-1] = 0.000001
        #         # sensing update
        #         # find the points closest to white/black lines
        #         if self.imageMap.white_lines:
        #             closeToWhite = filter(lambda p: min([math.hypot(p[0]-w[0],p[1]-w[1]) for w in self.imageMap.white_lines]) <= 4, self.particles)
        #         if self.imageMap.black_lines:
        #             closeToBlack = filter(lambda p: min([math.hypot(p[0]-b[0],p[1]-b[1]) for b in self.imageMap.black_lines]) <= 4, self.particles)
        #         for oldPt in oldGen:
        #             if closeToWhite or closeToBlack:
        #                 if oldPt in closeToWhite:
        #                     oldPt[2] = whiteExpected
        #                 elif oldPt in closeToBlack:
        #                     oldPt[2] = blackExpected
        #                 else:
        #                     oldPt[2] = neutralExpected
        #                 # set probabilities depending on difference
        #                 # between point's expected light and
        #                 # robot's observed light
        #                 if D.sensors[1] >= 400:
        #                     if oldPt[2] == blackExpected:
        #                         oldPt[-1] *= 0.1
        #                     elif oldPt[2] == neutralExpected:
        #                         oldPt[-1] *= 0.7
        #                 else:
        #                     if oldPt[2] == whiteExpected:
        #                         oldPt[-1] *= 0.3
        #                     elif oldPt[2] == blackExpected:
        #                         oldPt[-1] *= 0.7
        #         oldProbs = [p[-1] for p in oldGen]
        #         sumProb = math.fsum(oldProbs)
        #         if sumProb <= 0.0001:
        #             # if all the points are very unlikely, just
        #             # start over with a new set of points
        #             self.particles = []
        #         else:
        #             # begin populating new generation with copies
        #             # from old generation, based on the points'
        #             # probabilities
        #             newGen = []
        #             newProbs = []
        #             cumulativeProb = [math.fsum(oldProbs[i::-1]) for i in
        #                               xrange(len(oldProbs))]
        #             counter = 0
        #             for n in xrange(len(self.particles)):
        #                 # step approach
        #                 newGen.append(oldGen[counter][:-1])
        #                 while (n*sumProb/len(self.particles) >
        #                         cumulativeProb[counter]):
        #                     counter += 1
        #             newSumProb = math.fsum([p[-1] for p in newGen])
        #             # add some noise to each new point
        #             self.particles = map(lambda p: [
        #                                 random.gauss(p[0],self.xyNoise),
        #                                 random.gauss(p[1],self.xyNoise),
        #                                 p[2],
        #                                 p[-1]/newSumProb
        #                                 ],
        #                               newGen)

        self.particle_info_update()
        image = self.display_update(image)

        # We could do more processing (eg OpenCV) here if we wanted
        # to, but for now let's just display the window.
        self.resize(self.width, self.height)
        self.imageBox.setPixmap(image)

        # Status bar displays charge level by default
        if (not self.statusBar().currentMessage() or 
                "Charge" in self.statusBar().currentMessage()):
            self.statusBar().showMessage("Charge: " + D.chargeLevel)

    def mcl_update(self):
        """the MCL algorithm"""
        if not self.particles:
            self.particles = [Particle(random.randint(0, self.width-1),
                                       random.randint(0, self.height-1),
                                       random.randint(0, 360),
                                       1.0 / self.numParticles)
                              for i in xrange(self.numParticles)]
        elif self.recent_move:
            displacement = [m - n for m, n in self.trail[-1], self.trail[-2]]
            # change particle weights based on updates from robot
            for p in self.particles:
                # motion update
                p.displace(*displacement)
                if abs(p.x) > D.width/2 or abs(p.y) > D.height/2:
                    p.prob *= 0.01
                # sensor update
                pass # TODO
            oldSumProb = math.fsum([p.prob for p in self.particles])
            if oldSumProb <= 0.01:
                # if all the points are really unlikely, just start over
                self.particles = []
            else:
                # resampling creates an entirely new list of particles
                # based on the probabilities from the old generation
                newGen = []
                cumulativeProb = [math.fsum(self.particles[:i].prob)
                  for i in xrange(self.numParticles)]
                for i in xrange(self.numParticles):
                    # step through probability "blocks" of particle list
                    step = float(i) / self.numParticles
                    while step > cumulativeProb[0]:
                        del cumulativeProb[0]
                        del self.particles[0]
                    newGen.append(self.particles[0])
                newSumProb = math.fsum([p.prob for p in newGen])
                while newGen:
                    # resampled particles' probabilities normalized
                    self.particles.append(
                      newGen.pop().finish_resample(newSumProb))

    def particle_info_update(self):
        """Updates particle info widget box"""
        if self.make_mcl == 0:
            self.particle_layout.setCurrentWidget(self.off_label)
        elif self.make_mcl == 1:
            self.particle_layout.setCurrentWidget(self.hide_label)
        elif self.make_mcl == 2:
            to_clear = [
                    self.particle_position_value.setText,
                    self.particle_theta_value.setText,
                    self.particle_weight_value.setText,
                    self.particle_sensor_back_left_position.setText,
                    self.particle_sensor_front_left_position.setText,
                    self.particle_sensor_front_right_position.setText,
                    self.particle_sensor_back_right_position.setText,
                    self.particle_sensor_back_left_value.setText,
                    self.particle_sensor_front_left_value.setText,
                    self.particle_sensor_front_right_value.setText,
                    self.particle_sensor_back_right_value.setText
                ]
            if self.selected_particle:
                self.particle_position_value.setText(
                  str((self.selected_particle.x, self.selected_particle.y)))
                self.particle_theta_value.setText(
                  str(self.selected_particle.theta))
                self.particle_weight_value.setText(
                  str(self.selected_particle.prob)[:10])
                try: # weird and probably bad code
                    self.particle_sensor_back_left_position.setText(
                      str((self.selected_particle.sensors[0].x,
                          self.selected_particle.sensors[0].y)))
                    self.particle_sensor_front_left_position.setText(
                      str((self.selected_particle.sensors[1].x,
                          self.selected_particle.sensors[1].y)))
                    self.particle_sensor_front_right_position.setText(
                      str((self.selected_particle.sensors[2].x,
                          self.selected_particle.sensors[2].y)))
                    self.particle_sensor_back_right_position.setText(
                      str((self.selected_particle.sensors[3].x,
                          self.selected_particle.sensors[3].y)))
                    self.particle_sensor_back_left_value.setText(
                      str(self.selected_particle.sensors[0].light))
                    self.particle_sensor_front_left_value.setText(
                      str(self.selected_particle.sensors[1].light))
                    self.particle_sensor_front_right_value.setText(
                      str(self.selected_particle.sensors[2].light))
                    self.particle_sensor_back_right_value.setText(
                      str(self.selected_particle.sensors[3].light))
                except IndexError:
                    for f in to_clear[3:]: f("")
            else:
                for f in to_clear: f("")
            self.particle_layout.setCurrentWidget(self.show_widget)

    def display_update(self, image):
        """Updates display image"""
        painter = QtGui.QPainter()
        painter.begin(image)
        # drawing axes
        if self.axesCheckbox.isChecked():
            painter.setPen(QtGui.QColor.fromHsv(240, 63, 63))
            painter.setBrush(QtGui.QColor.fromHsv(240, 63, 63))
            painter.drawLine(self.width/2, 0, self.width/2, self.height)
            painter.drawLine(0, self.height/2, self.width, self.height/2)
        # drawing grid
        if self.gridCheckbox.isChecked():
            spacing = 20
            painter.setPen(QtGui.QColor.fromHsv(240,63,63))
            painter.setBrush(QtGui.QColor.fromHsv(240,63,63))
            # grid is centered on origin
            for x in xrange((self.width/2)%spacing, self.width, spacing):
                for y in xrange((self.height/2)%spacing, self.height, spacing):
                    painter.drawEllipse(x, y, 1, 1)
        # drawing particles
        if self.make_mcl == 2:
            self.particleRadius = 2
            if self.particleDetailCheckbox.isChecked():
                # draws particle sensor locations and particle heading
                self.particleRadius = 3
                for p in self.particles:
                    painter.setPen("black")
                    sensorDraw = p.get_sensor_global_position()
                    for s in sensorDraw:
                        painter.drawEllipse(*s, w=1, h=1)
                    if p == self.selected_particle:
                        painter.setPen("white")
                        distance_from_particle = self.particleRadius + 5
                        particle_pointer_length = 6
                    else:
                        painter.setPen("black")
                        distance_from_particle = self.particleRadius + 3
                        particle_pointer_length = 4
                    xDistance = distance_from_particle * math.sin(
                      math.radians(p.theta + 90.0))
                    yDistance = distance_from_particle * math.cos(
                      math.radians(p.theta + 90.0))
                    xLength = xDistance + particle_pointer_length*math.sin(
                      math.radians(p.theta + 90.0))
                    yLength = yDistance + particle_pointer_length*math.cos(
                      math.radians(p.theta + 90.0))
                    pointer = QtCore.QLineF(xDistance + p.x,
                                            yDistance + p.y,
                                            xLength + p.x,
                                            yLength + p.y)
                    painter.drawLine(pointer)
            selected_radius = self.particleRadius + 2
            if self.particleColoringCheckbox.isChecked():
                hue = 0
                for p in self.particles:
                    # the color of one point is based on how many
                    # other points are close to it, "close" here
                    # being defined very arbitrarily
                    near = 10 # pixels
                    numClosePoints = len(filter(
                      lambda q: near >= abs(q.x - p.x) and
                                near >= abs(q.y - p.y), self.particles))
                    hue = 300 * (1 - numClosePoints/(self.numParticles*0.75))
                    if hue < 0.0: hue = 0
                    if hue > 300.0: hue = 300
                    if p == self.selected_particle:
                        color = QtGui.QColor.fromHsv(hue, 255, 255)
                        painter.setPen("black")
                        painter.setBrush(color)
                        painter.drawEllipse(QtCore.QPoint(p.x, p.y), 
                          selected_radius, selected_radius)
                    else:
                        color = QtGui.QColor.fromHsv(hue, 255, 180)
                        painter.setPen(color)
                        painter.setBrush(color)
                        painter.drawEllipse(QtCore.QPoint(p.x, p.y), 
                          self.particleRadius, self.particleRadius)
            else:
                for p in self.particles:
                    if p == self.selected_particle:
                        painter.setPen(QtGui.QColor("black"))
                        painter.setBrush(QtGui.QColor("red"))
                        painter.drawEllipse(QtCore.QPoint(p.x, p.y), 
                          selected_radius, selected_radius)
                    else:
                        painter.setPen(QtGui.QColor("darkGray"))
                        painter.setBrush(QtGui.QColor("darkGray"))
                        painter.drawEllipse(QtCore.QPoint(p.x, p.y), 
                          self.particleRadius, self.particleRadius)
        # drawing robot trail
        if self.make_trail == 2 and len(self.trail) >= 2:
            painter.setPen(QtGui.QColor(255, 0, 0))
            painter.setBrush(QtGui.QColor(255, 0, 0))
            for p in xrange(1,len(self.trail)):
                painter.drawLine(self.trail[p-1][0], self.trail[p-1][1],
                                 self.trail[p][0], self.trail[p][1])
        # drawing map features
        if self.imageMap.white_lines:
            painter.setPen(QtGui.QColor(255, 255, 255))
            painter.setBrush(QtGui.QColor(255, 255, 255))
            for p in self.imageMap.white_lines:
                painter.drawPoint(p[0], p[1])
        if self.imageMap.black_lines:
            painter.setPen(QtGui.QColor(0, 0, 0))
            painter.setBrush(QtGui.QColor(0, 0, 0))
            for p in self.imageMap.black_lines:
                painter.drawPoint(p[0], p[1])
        # drawing robot location and pointer
        if self.robotMarkerCheckbox.isChecked():
            markerRadius = 8
            painter.setPen(QtGui.QColor("black"))
            painter.setBrush(QtGui.QColor("lightGray"))
            painter.drawEllipse(
              QtCore.QPoint(self.trail[-1][0], self.trail[-1][1]),
              markerRadius, markerRadius)
            painter.setPen(QtGui.QColor(0,255,0))
            distanceFromMarker = markerRadius + 3
            pointerLength = 8
            xDistance = distanceFromMarker * math.sin(
              math.radians(float(self.thetaValue.text()) + 90.0))
            yDistance = distanceFromMarker * math.cos(
              math.radians(float(self.thetaValue.text()) + 90.0))
            xLength = xDistance + pointerLength*math.sin(
              math.radians(float(self.thetaValue.text()) + 90.0))
            yLength = yDistance + pointerLength*math.cos(
              math.radians(float(self.thetaValue.text()) + 90.0))
            pointer = QtCore.QLineF(xDistance + self.trail[-1][0],
                                    yDistance + self.trail[-1][1],
                                    xLength + self.trail[-1][0],
                                    yLength + self.trail[-1][1])
            painter.drawLine(pointer)
        painter.end()
        return image

    ###
    # MCL parameter slots
    ###
    def noise_slider_change(self):
        """Slot for noise sliders"""
        sender = self.sender()
        if sender == self.xyNoiseSlider:
            self.xyNoise = self.xyNoiseSlider.value() / 10.0
            self.xyNoiseField.setText(str(self.xyNoise))
        elif sender == self.thetaNoiseSlider:
            self.thetaNoise = self.thetaNoiseSlider.value() / 10.0
            self.thetaNoiseField.setText(str(self.thetaNoise))

    def noise_field_change(self):
        """Slot for noise text fields"""
        sender = self.sender()
        if sender == self.xyNoiseField:
            self.xyNoise = float(self.xyNoiseField.text())
            self.xyNoiseSlider.blockSignals(True)
            self.xyNoiseSlider.setSliderPosition(10 * self.xyNoise)
            self.xyNoiseSlider.blockSignals(False)
        elif sender == self.thetaNoiseField:
            self.thetaNoise = float(self.thetaNoiseField.text())
            self.thetaNoiseSlider.blockSignals(True)
            self.thetaNoiseSlider.setSliderPosition(10 * self.thetaNoise)
            self.thetaNoiseSlider.blockSignals(False)

    def num_particles_set(self):
        """receives editingFinished signal from numParticlesField"""
        self.numParticles = int(self.numParticlesField.text())

    ###
    # Light sensor slot
    ###
    def light_threshold_set(self):
        """Light threshold field widgets are connected to this"""
        sender = self.sender()
        corresponding = {
            self.backLeftLowerField: 0,
            self.frontLeftLowerField: 1,
            self.frontRightLowerField: 2,
            self.backRightLowerField: 3,
            self.backLeftUpperField: 4,
            self.frontLeftUpperField: 5,
            self.frontRightUpperField: 6,
            self.backRightUpperField: 7
        }
        self.thresholds[corresponding[sender]] = int(sender.text())

    ###
    # Image I/O slots
    ###
    def save_image(self):
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
    
    def open_image(self):
        toLoad = QtGui.QImage()
        dialog = QtGui.QFileDialog(self)
        dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
        dialog.setViewMode(QtGui.QFileDialog.Detail)
        fname, _ = dialog.getOpenFileName(
          self, "Open image (features must be black and/or white)",
          "/home/robotics/Desktop/", "*.png")
        if toLoad.load(fname):
            self.width = toLoad.width()
            self.height = toLoad.height()
            self.imageMap.white_lines = []
            self.imageMap.black_lines = []
            self.setWindowTitle('RobotBox - ' + fname)
            self.erase_trail()
            self.erase_mcl()
            # add black/white pixels to their lists
            for x in xrange(self.width):
                for y in xrange(self.height):
                    if (QtGui.QColor(toLoad.pixel(x,y)) ==
                            QtGui.QColor("white")):
                        self.imageMap.white_lines.append((x,y))
                    elif (QtGui.QColor(toLoad.pixel(x,y)) ==
                            QtGui.QColor("black")):
                        self.imageMap.black_lines.append((x,y))
            self.statusBar().showMessage("Image " + fname + " opened", 3000)
        else:
            self.statusBar().showMessage("Failed to open image", 3000)
    
    def clear_image(self):
        self.width = DEFAULT_SIZE
        self.height = DEFAULT_SIZE
        self.imageMap.white_lines = []
        self.imageMap.black_lines = []
        self.setWindowTitle('RobotBox')
        self.erase_trail()
        self.erase_mcl()
        self.statusBar().showMessage("Cleared image", 3000)

    ###
    # Drawing slots
    ###
    def set_trail(self):
        """Slot for Trail radio buttons"""
        sender = self.sender()
        if sender == self.showTrailButton:
            self.eraseTrailButton.setEnabled(True)
            self.make_trail = 2
        elif sender == self.hideTrailButton:
            self.eraseTrailButton.setEnabled(True)
            self.make_trail = 1
        elif sender == self.noTrailButton:
            self.erase_trail()
            self.eraseTrailButton.setEnabled(False)
            self.make_trail = 0
    
    def erase_trail(self): self.trail = []

    def set_mcl(self):
        """Slot for MCL radio buttons"""
        sender = self.sender()
        if sender == self.showMCLButton:
            self.eraseMCLButton.setEnabled(True)
            self.make_mcl = 2
        elif sender == self.hideMCLButton:
            self.eraseMCLButton.setEnabled(True)
            self.make_mcl = 1
        elif sender == self.noMCLButton:
            self.erase_mcl()
            self.eraseMCLButton.setEnabled(False)
            self.make_mcl = 0

    def erase_mcl(self): self.particles = []

    def offset_slider_change(self):
        """Slot for position offset sliders"""
        sender = self.sender()
        if sender == self.x_offset_slider:
            self.x_offset = self.x_offset_slider.value()
            self.x_offset_field.setText(str(self.x_offset))
        elif sender == self.y_offset_slider:
            self.y_offset = self.y_offset_slider.value()
            self.y_offset_field.setText(str(self.y_offset))
        elif sender == self.theta_offset_slider:
            self.theta_offset = self.theta_offset_slider.value()
            self.theta_offset_field.setText(str(self.theta_offset))
            self.pivot = (D.xPrevious, D.yPrevious)
        self.erase_trail()
        self.erase_mcl()

    def offset_field_change(self):
        """Slot for position offset text fields"""
        sender = self.sender()
        if sender == self.x_offset_field:
            self.x_offset = int(self.x_offset_field.text())
            self.x_offset_slider.blockSignals(True)
            self.x_offset_slider.setSliderPosition(self.x_offset)
            self.x_offset_slider.blockSignals(False)
        elif sender == self.y_offset_field:
            self.y_offset = int(self.y_offset_field.text())
            self.y_offset_slider.blockSignals(True)
            self.y_offset_slider.setSliderPosition(self.y_offset)
            self.y_offset_slider.blockSignals(False)
        elif sender == self.theta_offset_field:
            self.theta_offset = int(self.theta_offset_field.text())
            self.theta_offset_slider.blockSignals(True)
            self.theta_offset_slider.setSliderPosition(self.theta_offset)
            self.theta_offset_slider.blockSignals(False)
            self.pivot = (D.xPrevious, D.yPrevious)
        self.erase_trail()
        self.erase_mcl()

    ###
    # Position slot
    ###
    def position_reset(self):
        """Position reset buttons connect to this"""
        sender = self.sender()
        if sender == self.x_reset_button or sender == self.all_reset_button:
            self.xDiff = D.x
        if sender == self.y_reset_button or sender == self.all_reset_button:
            self.yDiff = D.y
        if (sender == self.theta_reset_button or 
                sender == self.all_reset_button):
            self.thetaDiff = D.theta
            self.pivot = (D.xPrevious,D.yPrevious)
        self.erase_trail()
        self.erase_mcl()


def upper_left_origin(pt, w, h):
    """converts points to coordinate system with top-left corner as
    origin and bottom/right being positive directions"""
    x = pt[0] + w/2
    y = -pt[1] + h/2
    return (x, y)


def center_origin(pt, w, h):
    """converts points to coordinate system with centered origin and
    right/up being positive directions"""
    x = pt[0] - w/2
    y = -pt[1] + h/2
    return (x, y)


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


if __name__ == '__main__':
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
