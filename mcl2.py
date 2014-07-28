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
SCALE = 1.0 # how many distance units for one pixel? TODO
WIDGET_SPACING = 10 # pixels of space between widgets
DEFAULT_SIZE = 300 # side length of default image


class Data: pass    # empty class for a generic data holder
D = Data()
D.use_gui_control = False


class Map(object):

    """An image file interpreted as a map.

    Instance variables:
    black_lines -- black pixels representing black lines on floor
    white_lines -- white pixels representing white lines on floor

    Methods:
    light_of_point -- gets expected light value of given point
    """

    black_value = 0
    neutral_value = 200
    white_value = 1600

    def __init__(self):
        self.black_lines = []
        self.white_lines = []

    def __nonzero__(self):
        return False if self == Map() else True

    def light_of_point(self, point):
        """Gets expected light value of given point"""
        if point in self.black_lines:
            return self.black_value
        elif point in self.white_lines:
            return self.white_lines
        else:
            return self.neutral_value


class Sensor(object):

    """Represents IR light sensor on a (possibly hypothetical) robot.

    Various measurements of the robot are as follows (warning: sample
    size of 1):
    From Bluetooth receiver to front circle thingy - 6.4 cm
    From the word "activate" on the bottom to
        back left sensor - 16.5 cm
        front left sensor - 16.4 cm
        front right sensor - 16.3 cm
        back right sensor - 16.5 cm
    To the front wheel from
        back left sensor - 17.8 cm
        front left sensor - 4.8 cm
        front right sensor - 4.7 cm
        back right sensor - 18.0 cm

    Instance variables:
    x -- the sensor's x-coordinate relative to the robot's center
    y -- the sensor's y-coordinate
    light -- the light value the sensor sees
    """

    def __init__(self, x, y, light=-1):
        """Constructs one sensor."""
        self.x = x
        self.y = y
        self.light = light


class Robot(object):

    """Represents the Robot on the map.

    Instance variables:
    x, y -- the coordinates of the robot
    theta -- the angle at which the robot would move forward,
        relative to the horizontal
    x_previous, y_previous -- reported coordinates from previous GUI
        update
    chargeLevel -- displays robot charge level
    sensors -- an array of four sensors

    Class variables:
    sensor_angles -- an array of the angles of the four sensors
        relative to the direction the robot is facing
    sensor_initial_position -- an array of the initial coordinates of
        the four sensors

    Methods:
    update_sensor_positions -- update sensors' coordinates
    update_sensor_values -- update sensors' values using actual robot's
        sensor values
    get_sensor_global_position -- converts sensor's relative coordinates
        to global coordinates
    """

    # The angles between a line from the center to the front and a line
    # from the center to the sensor
    sensor_angles = [66, 16, -16, -66]
    # The Cartesian coordinates of the sensors when theta = 0 deg
    sensor_initial_position = []
    for args in [
      (6.71, 15.07), (15.86, 4.55), (-15.86, -4.55), (-6.71, -15.07)]:
        sensor_initial_position.append(Sensor(*args))
    recent_move = False

    def __init__(self):
        """Initialize robot with values"""
        self.x = {'raw': 0.0, 'display': 0.0, 'draw': 0.0, 'offset': 0,
                  'diff': 0.0, 'prev': 0.0}
        self.y = {'raw': 0.0, 'display': 0.0, 'draw': 0.0, 'offset': 0,
                  'diff': 0.0, 'prev': 0.0}
        self.t = {'raw': 0.0, 'display': 0.0, 'offset': 0, 'diff': 0.0}
        self.pivot = ()
        self.charge_level = ""
        self.sensors = self.sensor_initial_position

    def update_display(self):
        """Updates the coordinates displayed in the "Position" tab,
        which are also the cartesian coordinates of the robot in the
        display image"""
        # convert theta to degrees and set bounds
        # 0 degrees arbitrarily defined to be the horizontal and
        # theta increases in the counterclockwise direction
        theta_display = self.t['offset'] + self.t['raw'] - self.t['diff']
        while theta_display > 360: theta_display -= 360
        while theta_display < 0: theta_display += 360
        if self.t['diff'] or self.t['offset']:
            if self.pivot:
                distance = math.hypot(
                  self.x['raw'] - self.pivot[0], self.y['raw'] - self.pivot[1])
                pivotAngle = math.atan2(
                  self.y['raw'] - self.pivot[1], self.x['raw'] - self.pivot[0])
                pivotAngle += 2*math.pi
                x_actual = self.pivot[0] + distance*math.cos(
                  pivotAngle + math.radians(self.t['offset']))
                y_actual = self.pivot[1] + distance*math.sin(
                  pivotAngle + math.radians(self.t['offset']))
            else:
                self.pivot = (self.x['previous'], self.y['previous'])
        else:
            x_actual = self.x['raw']
            y_actual = self.y['raw']
        # if USE_CM = 1, values are converted here
        self.x['display'] = (100.0**USE_CM*(x_actual-self.x['diff']) +
          self.x['offset']/SCALE)
        self.y['display'] = (100.0**USE_CM*(y_actual-self.y['diff']) +
          self.y['offset']/SCALE)
        self.t['display'] = theta_display
        
    def update_draw(self, w, h):
        """Updates coordinates of robot marker passed into whatever
        draws the graphics
        """
        origin = (w/2, h/2)
        self.x['draw'] = origin[0] + self.x['display']*SCALE
        self.y['draw'] = origin[1] - self.y['display']*SCALE

    def update_sensor_positions(self):
        """Update sensor coordinates using the robot's coordinates"""
        for s in self.sensors:
            distance = math.hypot(s.x, s.y)
            angle = self.sensor_angles[self.sensors.index(s)]-self.t['display']
            s.x = math.cos(math.radians(angle)) * distance
            s.y = math.sin(math.radians(angle)) * distance

    def update_sensor_values(self, values=[], virtual=False, image_map=False):
        """Update sensor light values"""
        if image_map and virtual:
            for s in self.sensors:
                s.light = image_map.light_of_point((s.x, s.y))
        elif values:
            for i in xrange(4):
                self.sensors[i].light = values[i]

    def get_sensor_global_position(self):
        """Use the robot's x, y, theta to convert its Sensor object
        coordinates to global coordinates
        """
        self.update_sensor_positions()
        return [Sensor(s.x+self.x['display'], s.y+self.y['display'], s.light)
                for s in self.sensors]

    def get_sensor_draw_position(self):
        self.update_sensor_positions()
        return [Sensor(s.x+self.x['draw'], s.y+self.y['draw'], s.light)
                for s in self.sensors]


class Particle(Robot): 

    """Represents a single MCL particle, which is a hypothetical robot.

    New instance variable:
    probability -- the probability that the robot's location
        matches this particle's location

    New class variable:
    selected -- whether there is a double-clicked particle

    New and overriding methods:
    displace -- a neat way to change x, y, theta
    update_sensor_values -- update sensors' coordinates and values
        based on given map
    finish_resample -- the last thing a particle needs to do before it
        can really be part of a new generation
    """

    # Stores the selected particle
    selected = None

    def __init__(self, x, y, t, probability):
        """Construct the generic MCL particle"""
        self.x = {'display': x, 'draw': 0.0, 'prev': 0.0}
        self.y = {'display': y, 'draw': 0.0, 'prev': 0.0}
        self.t = {'display': t}
        self.sensors = self.sensor_initial_position
        self.prob = probability

    def update_display(self, x, y, t, direction):
        """Moves the particle as specified by the arguments"""
        total_disp = ((x**2)+(y**2))**.5
        self.t['display'] += t
        while self.t['display'] > 360: self.t['display'] -= 360
        while self.t['display'] < 360: self.t['display'] += 360
        my_t = self.t['display'] 
        my_t_in_rad = math.radians( my_t )

        if not D.use_gui_control: # TODO
            direction = "Forward"

        x = math.cos( my_t_in_rad )*total_disp
        y = math.sin( my_t_in_rad )*total_disp

        if direction=="Forward":
            self.x['display'] += x
            self.y['display'] += y
        if direction=="Backward":
            self.x['display'] -= x
            self.y['display'] -= y

    # def update_sensor_values(self, image_map):
    #     """Update sensor light values based on map"""
    #     super(Particle, self).update_sensor_values(
    #       virtual=True, image_map=image_map)

    def finish_resample(self, xy_noise, t_noise, psum):
        """Adds noise to x, y, theta, and divides prob by sum"""
        x_diff = random.gauss(0, xy_noise)
        self.x['display'] += x_diff
        self.x['draw'] += x_diff
        y_diff = random.gauss(0, xy_noise)
        self.y['display'] += y_diff
        self.y['draw'] += y_diff
        t_diff = random.gauss(0, t_noise)
        self.t['display'] += t_diff
        self.prob /= psum


class RobotGUI(QtGui.QMainWindow):

    """The GUI

    Instance variables, not including widgets:
    width, height -- dimensions of display image
    imageMap -- image map of area
    make_trail -- toggle robot trail
    make_mcl -- toggle MCL
    x_offset, y_offset, theta_offset -- robot marker offset in pixels
    trail -- list of all past robot locations, from oldest to newest
    recent_move -- true if robot moved since last GUI update
    xDiff, yDiff, thetaDiff -- for "clearing" odometer
    pivot -- for theta offset/reset purposes
    thresholds -- user-input light thresholds
    numParticles -- how many MCL particles
    particles -- list of Particle objects
    xyNoise, thetaNoise -- MCL resampled points noise, as sigma of
        Gaussian distribution
    statusMessage -- the message in the window's bottom status bar
    redrawTimer -- redraws GUI with each cycle
    """

    def __init__(self):
        # Construct the parent class
        super(RobotGUI, self).__init__()
        # Initialize data values related to visuals/drawing
        self.width = DEFAULT_SIZE
        self.height = DEFAULT_SIZE
        self.virtual_robot_meter_step = 0.01
        self.virtual_robot_degree_step = 1.0
        self.imageMap = Map()
        self.make_trail = 0
        self.make_mcl = 0
        # Initialize data values related to location
        self.trail = []
        # Initialize light threshold array
        # First four are lower, last four are upper
        self.thresholds = [100, 180, 180, 150, 800, 1000, 1000, 1000]
        # Initialize data values related to MCL
        #self.numParticles = int(self.width * self.height * 0.01)
        self.numParticles = 1
        self.particles = []
        self.particleRadius = 0
        self.xyNoise = 0.0
        self.thetaNoise = 0.0
        # Initialize robot
        D.robot = Robot()
        # Initialize each section of the main window
        self.init_menu()
        self.init_central()
        self.init_left()
        self.init_right()
        self.setFocusPolicy(QtCore.Qt.ClickFocus)

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
        # positionGroup # TODO
        ###
        positionGroup = QtGui.QWidget(self)
        # coordinate display
        xLabel = QtGui.QLabel("X (" + USE_CM*"c" + "m): ", self)
        self.xValue = QtGui.QLabel(self)
        yLabel = QtGui.QLabel("Y (" + USE_CM*"c" + "m): ", self)
        self.yValue = QtGui.QLabel(self)
        thetaLabel = QtGui.QLabel("Theta (deg): ", self)
        self.thetaValue = QtGui.QLabel(self)
        # coordinate reset buttons
        self.x_reset_button = QtGui.QPushButton("Reset", self)
        self.x_reset_button.clicked.connect(self.position_reset)
        self.y_reset_button = QtGui.QPushButton("Reset", self)
        self.y_reset_button.clicked.connect(self.position_reset)
        self.theta_reset_button = QtGui.QPushButton("Reset", self)
        self.theta_reset_button.clicked.connect(self.position_reset)
        self.all_reset_button = QtGui.QPushButton("Reset all", self)
        self.all_reset_button.clicked.connect(self.position_reset)

        self.virtual_control_checkbox = QtGui.QCheckBox(
          "Use GUI/keyboard input as robot motion data", self)
        self.virtual_control_checkbox.toggled.connect(
          self.virtual_control_toggle)

        physical_group = QtGui.QWidget(self)
        physicalLayout = QtGui.QGridLayout()
        physical_group.setLayout(physicalLayout)
        physicalWidgetsToAdd = [
                             (xLabel, 1, 1, QtCore.Qt.AlignRight),
                        (self.xValue, 1, 2),
                (self.x_reset_button, 2, 1, 1, 2, QtCore.Qt.AlignHCenter),
                             (yLabel, 4, 1, QtCore.Qt.AlignRight),
                        (self.yValue, 4, 2),
                (self.y_reset_button, 5, 1, 1, 2, QtCore.Qt.AlignHCenter),
                         (thetaLabel, 1, 4, QtCore.Qt.AlignRight),
                    (self.thetaValue, 1, 5),
            (self.theta_reset_button, 2, 4, 1, 2, QtCore.Qt.AlignHCenter),
              (self.all_reset_button, 4, 4, 2, 2, QtCore.Qt.AlignHCenter),
              (self.virtual_control_checkbox, 7, 1, 1, 5)
        ]
        for args in physicalWidgetsToAdd: physicalLayout.addWidget(*args)
        # blank rows/columns for spacing between widgets
        physicalLayout.setColumnMinimumWidth(3, WIDGET_SPACING)
        physicalLayout.setRowMinimumHeight(6, WIDGET_SPACING)
        physicalLayout.setColumnStretch(0, 1)
        physicalLayout.setColumnStretch(6, 1)
        
        

        self.step_label = QtGui.QLabel("Step: ", self)
        self.step_meter_label = QtGui.QLabel("m", self)
        self.step_degree_label = QtGui.QLabel("deg", self)
        meter_validator = QtGui.QDoubleValidator(1.0/(100.0**USE_CM), 50.0/(100.0**USE_CM), 2, self)
        self.step_meter_field = QtGui.QLineEdit(
          str(self.virtual_robot_meter_step))
        self.step_meter_field.editingFinished.connect(
          self.virtual_robot_step_set)
        self.step_meter_field.setValidator(meter_validator)
        self.step_meter_field.setFixedWidth(40)
        degree_validator = QtGui.QDoubleValidator(1.0, 90.0, 2, self)
        self.step_degree_field = QtGui.QLineEdit(
          str(self.virtual_robot_degree_step))
        self.step_degree_field.editingFinished.connect(
          self.virtual_robot_step_set)
        self.step_degree_field.setValidator(degree_validator)
        self.step_degree_field.setFixedWidth(40)

        self.forward_button = QtGui.QPushButton("Forward", self)
        self.forward_button.setAutoRepeat(True)
        self.forward_button.clicked.connect(self.virtual_move)
        self.backward_button = QtGui.QPushButton("Backward", self)
        self.backward_button.setAutoRepeat(True)
        self.backward_button.clicked.connect(self.virtual_move)
        self.turn_left_button = QtGui.QPushButton("Turn left", self)
        self.turn_left_button.setAutoRepeat(True)
        self.turn_left_button.clicked.connect(self.virtual_move)
        self.turn_right_button = QtGui.QPushButton("Turn right", self)
        self.turn_right_button.setAutoRepeat(True)
        self.turn_right_button.clicked.connect(self.virtual_move)
        self.virtual_group = QtGui.QWidget(self)
        virtualLayout = QtGui.QGridLayout()
        self.virtual_group.setLayout(virtualLayout)
        virtualWidgetsToAdd = [
                (self.step_label, 2, 2, QtCore.Qt.AlignRight),
                (self.step_meter_field, 2, 4, QtCore.Qt.AlignHCenter),
                (self.step_meter_label, 2, 5, QtCore.Qt.AlignHCenter),
                (self.step_degree_field, 2, 7, QtCore.Qt.AlignHCenter),
                (self.step_degree_label, 2, 8, QtCore.Qt.AlignHCenter),
                (self.forward_button, 4, 4, 1, 2, QtCore.Qt.AlignHCenter),
              (self.turn_left_button, 4, 1, 2, 2, QtCore.Qt.AlignHCenter),
             (self.turn_right_button, 4, 7, 2, 2, QtCore.Qt.AlignHCenter),
               (self.backward_button, 5, 4, 1, 2, QtCore.Qt.AlignHCenter),
        ]
        for args in virtualWidgetsToAdd: virtualLayout.addWidget(*args)
        virtualLayout.setColumnMinimumWidth(3, WIDGET_SPACING)
        virtualLayout.setColumnMinimumWidth(6, WIDGET_SPACING)
        virtualLayout.setColumnStretch(0, 1) # TODO
        virtualLayout.setColumnStretch(20, 1)

        physicalLayout.addWidget(self.virtual_group, 8, 1, 1, 5)

        # Assembling layout for positionGroup
        positionLayout = QtGui.QGridLayout()
        positionGroup.setLayout(physicalLayout)
        # positionLayout.addWidget(physical_group, 1, 1, QtCore.Qt.AlignHCenter)
        # positionLayout.addWidget(self.virtual_group, 3, 1, QtCore.Qt.AlignHCenter)
        # # increase stretch of border rows to keep the widgets close together
        # positionLayout.setRowStretch(4, 1)
        # positionLayout.setColumnStretch(0, 1)
        # positionLayout.setColumnStretch(2, 1)

        self.virtual_control_toggle()

        ###
        # lightGroup
        ###
        lightGroup = QtGui.QWidget(self)
        positionLabel = QtGui.QLabel("Position", self)
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
        self.backLeftPosition = QtGui.QLabel(self)
        self.frontLeftPosition = QtGui.QLabel(self)
        self.frontRightPosition = QtGui.QLabel(self)
        self.backRightPosition = QtGui.QLabel(self)
        self.backLeftValue = QtGui.QLabel(self)
        self.frontLeftValue = QtGui.QLabel(self)
        self.frontRightValue = QtGui.QLabel(self)
        self.backRightValue = QtGui.QLabel(self)

        # Light threshold entry
        validator = QtGui.QIntValidator(0, 9999, self)
        self.backLeftLowerField = QtGui.QLineEdit(
          str(self.thresholds[0]))
        self.frontLeftLowerField = QtGui.QLineEdit(
          str(self.thresholds[1]))
        self.frontRightLowerField = QtGui.QLineEdit(
          str(self.thresholds[2]))
        self.backRightLowerField = QtGui.QLineEdit(
          str(self.thresholds[3]))
        self.backLeftUpperField = QtGui.QLineEdit(
          str(self.thresholds[4]))
        self.frontLeftUpperField = QtGui.QLineEdit(
          str(self.thresholds[5]))
        self.frontRightUpperField = QtGui.QLineEdit(
          str(self.thresholds[6]))
        self.backRightUpperField = QtGui.QLineEdit(
          str(self.thresholds[7]))
        light_fields = [self.backLeftLowerField, self.frontLeftLowerField,
                        self.frontRightLowerField, self.backRightLowerField,
                        self.backLeftUpperField, self.frontLeftUpperField,
                        self.frontRightUpperField, self.backRightUpperField]
        for f in light_fields:
            f.editingFinished.connect(self.light_threshold_set)
            f.setValidator(validator)

        # Assembling layout for lightGroup
        lightLayout = QtGui.QGridLayout()
        lightGroup.setLayout(lightLayout)
        lightWidgetsToAdd = [
                        (positionLabel, 1, 3, QtCore.Qt.AlignHCenter),
                        (readingsLabel, 1, 5, QtCore.Qt.AlignHCenter),
                 (lowerThresholdsLabel, 1, 7, QtCore.Qt.AlignHCenter),
                 (upperThresholdsLabel, 1, 9, QtCore.Qt.AlignHCenter),
                        (backLeftLabel, 3, 1, QtCore.Qt.AlignRight),
                (self.backLeftPosition, 3, 3, QtCore.Qt.AlignHCenter),
                   (self.backLeftValue, 3, 5, QtCore.Qt.AlignHCenter),
              (self.backLeftLowerField, 3, 7, QtCore.Qt.AlignHCenter),
              (self.backLeftUpperField, 3, 9, QtCore.Qt.AlignHCenter),
                       (frontLeftLabel, 5, 1, QtCore.Qt.AlignRight),
               (self.frontLeftPosition, 5, 3, QtCore.Qt.AlignHCenter),
                  (self.frontLeftValue, 5, 5, QtCore.Qt.AlignHCenter),
             (self.frontLeftLowerField, 5, 7, QtCore.Qt.AlignHCenter),
             (self.frontLeftUpperField, 5, 9, QtCore.Qt.AlignHCenter),
                      (frontRightLabel, 7, 1, QtCore.Qt.AlignRight),
              (self.frontRightPosition, 7, 3, QtCore.Qt.AlignHCenter),
                 (self.frontRightValue, 7, 5, QtCore.Qt.AlignHCenter),
            (self.frontRightLowerField, 7, 7, QtCore.Qt.AlignHCenter),
            (self.frontRightUpperField, 7, 9, QtCore.Qt.AlignHCenter),
                       (backRightLabel, 9, 1, QtCore.Qt.AlignRight),
               (self.backRightPosition, 9, 3, QtCore.Qt.AlignHCenter),
                  (self.backRightValue, 9, 5, QtCore.Qt.AlignHCenter),
             (self.backRightLowerField, 9, 7, QtCore.Qt.AlignHCenter),
             (self.backRightUpperField, 9, 9, QtCore.Qt.AlignHCenter)
        ]
        for args in lightWidgetsToAdd: lightLayout.addWidget(*args)
        # border stretch
        lightLayout.setRowStretch(10, 1)
        # spacing rows
        lightLayout.setColumnMinimumWidth(2, WIDGET_SPACING)
        lightLayout.setColumnMinimumWidth(4, WIDGET_SPACING)
        lightLayout.setColumnMinimumWidth(6, WIDGET_SPACING) 
        lightLayout.setColumnMinimumWidth(8, WIDGET_SPACING)

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
                 (particle_position_label, 0, 0, QtCore.Qt.AlignHCenter),
            (self.particle_position_value, 1, 0, QtCore.Qt.AlignHCenter),
                    (particle_theta_label, 3, 0, QtCore.Qt.AlignHCenter),
               (self.particle_theta_value, 4, 0, QtCore.Qt.AlignHCenter),
                   (particle_weight_label, 6, 0, QtCore.Qt.AlignHCenter),
              (self.particle_weight_value, 7, 0, QtCore.Qt.AlignHCenter)
        ]
        for args in particle_info_widgets_to_add:
            particle_info_layout.addWidget(*args)
        particle_info_layout.setRowMinimumHeight(2, WIDGET_SPACING)
        particle_info_layout.setRowMinimumHeight(5, WIDGET_SPACING)
        particle_info_layout.setRowStretch(8, 1)
        particle_info_layout.setColumnStretch(0, 1)

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
            (particle_sensor_position_label, 0, 1,
              QtCore.Qt.AlignHCenter),
            (particle_sensor_value_label, 0, 2, QtCore.Qt.AlignHCenter),
            (particle_sensor_back_left_label, 1, 0),
            (particle_sensor_front_left_label, 2, 0),
            (particle_sensor_front_right_label, 3, 0),
            (particle_sensor_back_right_label, 4, 0),
            (self.particle_sensor_back_left_position, 1, 1,
              QtCore.Qt.AlignHCenter),
            (self.particle_sensor_front_left_position, 2, 1,
              QtCore.Qt.AlignHCenter),
            (self.particle_sensor_front_right_position, 3, 1,
              QtCore.Qt.AlignHCenter),
            (self.particle_sensor_back_right_position, 4, 1,
              QtCore.Qt.AlignHCenter),
            (self.particle_sensor_back_left_value, 1, 2,
              QtCore.Qt.AlignHCenter),
            (self.particle_sensor_front_left_value, 2, 2,
              QtCore.Qt.AlignHCenter),
            (self.particle_sensor_front_right_value, 3, 2,
              QtCore.Qt.AlignHCenter),
            (self.particle_sensor_back_right_value, 4, 2,
              QtCore.Qt.AlignHCenter)
        ]
        for args in particle_sensor_info_widgets_to_add:
            particle_sensor_info_layout.addWidget(*args)
        particle_sensor_info_layout.setColumnStretch(1, 1)
        particle_sensor_info_layout.setColumnStretch(2, 1)

        show_layout = QtGui.QGridLayout()
        self.show_widget.setLayout(show_layout)
        show_widgets_to_add = [(show_label, 1, 0, 1, 2),
                               (particle_info_group, 2, 0),
                               (particle_sensor_info_group, 2, 1)]
        for args in show_widgets_to_add: show_layout.addWidget(*args)
        show_layout.setRowStretch(3, 1)
        show_layout.setColumnStretch(1, 1)

        # Assembling layout for particle_group
        self.particle_layout = QtGui.QStackedLayout()
        particle_group.setLayout(self.particle_layout)
        particle_widgets_to_add = [self.off_label, self.hide_label,
                                   self.show_widget]
        for arg in particle_widgets_to_add: self.particle_layout.addWidget(arg)
        self.particle_layout.setCurrentWidget(self.off_label)

        ###
        # Top-level
        ###
        leftTabs.addTab(positionGroup, "Position")
        leftTabs.addTab(lightGroup, "IR sensors")
        leftTabs.addTab(particle_group, "Particles")
        leftTabs.setMinimumWidth(450)
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
            (self.robotMarkerCheckbox, 1, 1, 1, 2),
                   (self.axesCheckbox, 2, 1),
                   (self.gridCheckbox, 2, 2),
                          (trailLabel, 4, 1, 1, 2),
                (self.showTrailButton, 5, 1),
                (self.hideTrailButton, 5, 2),
                  (self.noTrailButton, 5, 3),
               (self.eraseTrailButton, 6, 1, 1, 3, QtCore.Qt.AlignLeft),
                            (MCLLabel, 8, 1, 1, 2),
                  (self.showMCLButton, 9, 1),
                  (self.hideMCLButton, 9, 2),
                    (self.noMCLButton, 9, 3),
                 (self.eraseMCLButton, 10, 1, 1, 3, QtCore.Qt.AlignLeft)
        ]
        for args in displayWidgetsToAdd: displayLayout.addWidget(*args)
        # blank rows/columns for spacing between widgets
        displayLayout.setRowMinimumHeight(2, WIDGET_SPACING)
        displayLayout.setRowMinimumHeight(6, WIDGET_SPACING)
        # border stretch
        displayLayout.setRowStretch(20, 1)

        # set initial button states
        self.noTrailButton.setChecked(True)
        self.noMCLButton.setChecked(True)
        self.eraseTrailButton.setEnabled(False)
        self.eraseMCLButton.setEnabled(False)

        ###
        # offsetGroup
        ###
        offsetGroup = QtGui.QWidget(self)

        # x
        x_offsetLabel = QtGui.QLabel("X", self)
        # x input field
        self.x_offset_field = QtGui.QLineEdit("0")
        self.x_offset_field.editingFinished.connect(self.offset_field_change)
        x_offsetValidator = QtGui.QIntValidator(
          -self.width/2, self.width/2, self)
        self.x_offset_field.setValidator(x_offsetValidator)
        # x slider
        self.x_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.x_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.x_offset_slider.setMinimum(-self.width)
        self.x_offset_slider.setMaximum(self.width)
        self.x_offset_slider.setTickInterval(self.width/10)
        self.x_offset_slider.setValue(0)
        self.x_offset_slider.valueChanged.connect(self.offset_slider_change)

        # y
        y_offsetLabel = QtGui.QLabel("Y", self)
        # y input field
        self.y_offset_field = QtGui.QLineEdit("0")
        self.y_offset_field.editingFinished.connect(self.offset_field_change)
        y_offsetValidator = QtGui.QIntValidator(
          -self.height / 2, self.height / 2, self)
        self.y_offset_field.setValidator(y_offsetValidator)
        # y slider
        self.y_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.y_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.y_offset_slider.setMinimum(-self.height)
        self.y_offset_slider.setMaximum(self.height)
        self.y_offset_slider.setTickInterval(self.height/10)
        self.y_offset_slider.setValue(0)
        self.y_offset_slider.valueChanged.connect(self.offset_slider_change)

        # theta
        theta_offsetLabel = QtGui.QLabel("Theta", self)
        # theta input field
        self.theta_offset_field = QtGui.QLineEdit("0")
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
        mclLayout.setRowStretch(20, 1)

        self.particleDetailCheckbox.setChecked(True)
        self.particleColoringCheckbox.setChecked(True)

        ###
        # Top-level
        ###
        rightTabs.addTab(displayGroup, "Draw")
        rightTabs.addTab(offsetGroup, "Offset")
        rightTabs.addTab(mclGroup, "MCL")
        # assembling dock
        self.rightDock = QtGui.QDockWidget("", self)
        self.rightDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        self.rightDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.rightDock.setTitleBarWidget(QtGui.QWidget(self))
        self.rightDock.setWidget(rightTabs)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.rightDock)

    def keyPressEvent(self, event):
        """Keyboard control of robot marker"""
        if self.hasFocus() and self.virtual_control_checkbox.isChecked():
            Robot.recent_move = True
            if event.key() == QtCore.Qt.Key_Up:
                x_displacement = self.virtual_robot_meter_step * math.cos(math.radians(D.robot.t['display']))
                y_displacement = self.virtual_robot_meter_step * math.sin(math.radians(D.robot.t['display']))
                D.robot.x['raw'] += x_displacement
                D.robot.y['raw'] += y_displacement
                Robot.recent_move = "Forward"
            elif event.key() == QtCore.Qt.Key_Down:
                x_displacement = self.virtual_robot_meter_step * math.cos(math.radians(D.robot.t['display']))
                y_displacement = self.virtual_robot_meter_step * math.sin(math.radians(D.robot.t['display']))
                D.robot.x['raw'] -= x_displacement
                D.robot.y['raw'] -= y_displacement
                Robot.recent_move = "Backward"
            elif event.key() == QtCore.Qt.Key_Left:
                D.robot.t['raw'] += self.virtual_robot_degree_step
            elif event.key() == QtCore.Qt.Key_Right:
                D.robot.t['raw'] -= self.virtual_robot_degree_step
            
    def mouseDoubleClickEvent(self, event):
        """Can double click on MCL particles to show info"""
        if self.make_mcl == 2 and self.imageBox == self.childAt(event.pos()):
            relative_click_pos = (
              event.x()-self.leftDock.width()-6,
              event.y() - (max(
                self.leftDock.height(), self.rightDock.height())-self.height)/2
            )
            closest_distance = self.particleRadius + 1
            for p in self.particles:
                distance = math.hypot(
                  relative_click_pos[0] - p.x['draw'],
                  relative_click_pos[1] - p.y['draw'])
                if distance < closest_distance:
                    # the closest particle to the double-click location
                    # will be selected
                    closest_distance = distance
                    Particle.selected = p
            if closest_distance == self.particleRadius + 1:
                Particle.selected = None

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
        sensor_global = D.robot.get_sensor_global_position()
        self.backLeftPosition.setText('({:.2f}, {:.2f})'.format(
          sensor_global[0].x, sensor_global[0].y))
        self.frontLeftPosition.setText('({:.2f}, {:.2f})'.format(
          sensor_global[1].x, sensor_global[1].y))
        self.frontRightPosition.setText('({:.2f}, {:.2f})'.format(
          sensor_global[2].x, sensor_global[2].y))
        self.backRightPosition.setText('({:.2f}, {:.2f})'.format(
          sensor_global[3].x, sensor_global[3].y))
        self.backLeftValue.setText(str(D.robot.sensors[0].light))
        self.frontLeftValue.setText(str(D.robot.sensors[1].light))
        self.frontRightValue.setText(str(D.robot.sensors[2].light))
        self.backRightValue.setText(str(D.robot.sensors[3].light))

        # updating position
        D.robot.update_display()
        self.thetaValue.setText('{:.1f}'.format(D.robot.t['display']))
        self.xValue.setText('{:.2f}'.format(D.robot.x['display']))
        self.yValue.setText('{:.2f}'.format(D.robot.y['display']))
        D.robot.update_draw(self.width, self.height)
        location = (D.robot.x['draw'], D.robot.y['draw'], D.robot.t['display'])
        if not self.trail or location != self.trail[-1]:
            self.trail.append(location)
            if not self.make_trail and len(self.trail) > 2:
                del self.trail[0]
        D.robot.update_sensor_positions()

        if self.make_mcl:
           self.mcl_update()
        self.particle_info_update()

        # We could do more processing (eg OpenCV) here if we wanted
        # to, but for now let's just display the window.
        image = self.display_update(image)
        self.resize(self.width, self.height)
        self.imageBox.setPixmap(image)

        # Status bar displays charge level by default
        if (not self.statusBar().currentMessage() or 
                "Charge" in self.statusBar().currentMessage()):
            self.statusBar().showMessage("Charge: " + D.robot.charge_level)

    def mcl_update(self):
        """the MCL algorithm"""
        if not self.particles:
            self.particles = [Particle(
                random.randint(-self.width/2, self.width/2),
                random.randint(-self.height/2, self.height/2),
                random.randint(0, 360),
                1.0 / self.numParticles)
              for i in xrange(self.numParticles)]
        else:
            for p in self.particles:
                p.update_draw(self.width, self.height)
                p.update_sensor_positions()
                p.update_sensor_values(virtual=D.use_gui_control, image_map=self.imageMap)
        if Robot.recent_move:
            # change particle weights based on updates from robot
            for p in self.particles:
                # motion update
                p.update_display(self.trail[-1][0]-self.trail[-2][0],
                                 self.trail[-1][1]-self.trail[-2][1], 
                                 self.trail[-1][2]-self.trail[-2][2],
                                 Robot.recent_move)
                if abs(p.x['display']) > self.width/2 or abs(p.y['display']) > self.height/2:
                    p.prob *= 0.01
                # sensor update
                # light_difference = max(map(lambda observed, expected: abs(observed.light-expected.light), D.robot.sensors, p.sensors))
                # if light_difference > 0:
                #     p.prob *= 1.0/light_difference
            Robot.recent_move = False
            oldSumProb = math.fsum([p.prob for p in self.particles])
            if oldSumProb <= 0.01:
                # if all the points are really unlikely, just start over
                self.particles = []
            else: # TODO
                # resampling creates an entirely new list of particles
                # based on the probabilities from the old generation
                newGen = []
                cumulativeProb = [math.fsum([p.prob for p in self.particles][i::-1]) for i in xrange(self.numParticles)]
                counter = 0
                for i in xrange(self.numParticles):
                    # step through probability "blocks" of particle list
                    newGen.append(self.particles[counter])
                    while (i*oldSumProb/self.numParticles >
                            cumulativeProb[counter]):
                        counter += 1
                newSumProb = math.fsum([p.prob for p in newGen])
                self.particles = newGen
                for p in self.particles:
                    # resampled particles' probabilities normalized
                    p.finish_resample(
                      self.xyNoise, self.thetaNoise, newSumProb)
        
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
            if Particle.selected is not None:
                self.particle_position_value.setText(str(
                  (Particle.selected.x['display'],
                   Particle.selected.y['display'])))
                self.particle_theta_value.setText(
                  str(Particle.selected.t['display']))
                self.particle_weight_value.setText(
                  '{:f}'.format(Particle.selected.prob))
                sensor_global = Particle.selected.get_sensor_global_position()
                self.particle_sensor_back_left_position.setText(
                  '({:.2f}, {:.2f})'.format(
                    sensor_global[0].x, sensor_global[0].y))
                self.particle_sensor_front_left_position.setText(
                  '({:.2f}, {:.2f})'.format(
                    sensor_global[1].x, sensor_global[1].y))
                self.particle_sensor_front_right_position.setText(
                  '({:.2f}, {:.2f})'.format(
                    sensor_global[2].x, sensor_global[2].y))
                self.particle_sensor_back_right_position.setText(
                  '({:.2f}, {:.2f})'.format(
                    sensor_global[3].x, sensor_global[3].y))
                self.particle_sensor_back_left_value.setText(
                  str(Particle.selected.sensors[0].light))
                self.particle_sensor_front_left_value.setText(
                  str(Particle.selected.sensors[1].light))
                self.particle_sensor_front_right_value.setText(
                  str(Particle.selected.sensors[2].light))
                self.particle_sensor_back_right_value.setText(
                  str(Particle.selected.sensors[3].light))
            else:
                for f in to_clear: f("")
            self.particle_layout.setCurrentWidget(self.show_widget)

    def display_update(self, image):
        """Updates display image"""
        painter = QtGui.QPainter()
        painter.begin(image)
        # drawing map features
        if self.imageMap:
            painter.setPen(QtGui.QColor(255, 255, 255))
            painter.setBrush(QtGui.QColor(255, 255, 255))
            for p in self.imageMap.white_lines:
                painter.drawPoint(p[0], p[1])
            painter.setPen(QtGui.QColor(0, 0, 0))
            painter.setBrush(QtGui.QColor(0, 0, 0))
            for p in self.imageMap.black_lines:
                painter.drawPoint(p[0], p[1])
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
            self.particleRadius = (3 if self.particleDetailCheckbox.isChecked()
              else 2)
            selected_radius = self.particleRadius + 2
            get_color = (self.calculate_color if
                self.particleColoringCheckbox.isChecked() else
                lambda x, y, z: QtGui.QColor("darkGray"))
            for p in self.particles:
                color = get_color(p, 255, 180)
                painter.setPen(color)
                painter.setBrush(color)
                painter.drawEllipse(QtCore.QPoint(p.x['draw'], p.y['draw']), 
                  self.particleRadius, self.particleRadius)
            if self.particleDetailCheckbox.isChecked():
                # draws particle sensor locations and particle heading
                for p in self.particles:
                    painter.setPen("black")
                    sensorDraw = p.get_sensor_draw_position()
                    for s in sensorDraw:
                        painter.drawPoint(s.x, s.y)
                    painter.drawLine(self.calculate_pointer(
                      p.x['draw'], p.y['draw'], p.t['display'],
                      self.particleRadius + 3, 4))
            if Particle.selected is not None:
                # determine color and draw particle
                color = self.calculate_color(Particle.selected, 255, 255)
                painter.setPen("black")
                painter.setBrush(color)
                painter.drawEllipse(
                  QtCore.QPoint(
                    Particle.selected.x['draw'], Particle.selected.y['draw']), 
                  selected_radius, selected_radius)
                # draw sensors
                painter.setPen("white")
                sensorDraw = Particle.selected.get_sensor_draw_position()
                for s in sensorDraw:
                    painter.drawPoint(s.x, s.y)
                # draw pointer
                painter.drawLine(self.calculate_pointer(
                  Particle.selected.x['draw'], Particle.selected.y['draw'],
                  Particle.selected.t['display'], self.particleRadius + 5, 6))
        # drawing robot trail
        if self.make_trail == 2 and len(self.trail) >= 2:
            painter.setPen(QtGui.QColor(255, 0, 0))
            painter.setBrush(QtGui.QColor(255, 0, 0))
            for p in xrange(1,len(self.trail)):
                painter.drawLine(self.trail[p-1][0], self.trail[p-1][1],
                                 self.trail[p][0], self.trail[p][1])
        # drawing robot marker
        if self.robotMarkerCheckbox.isChecked():
            markerRadius = 8
            painter.setPen(QtGui.QColor("black"))
            painter.setBrush(QtGui.QColor("lightGray"))
            painter.drawEllipse(
              QtCore.QPoint(self.trail[-1][0], self.trail[-1][1]),
              markerRadius, markerRadius)
            # sensors
            painter.setPen(QtGui.QColor(0,255,0))
            robot_sensor_draw = D.robot.get_sensor_draw_position()
            for s in robot_sensor_draw:
                painter.drawPoint(s.x, s.y)
            # pointer
            painter.drawLine(self.calculate_pointer(
              self.trail[-1][0], self.trail[-1][1],
              float(self.thetaValue.text()), markerRadius + 3, 8))
        painter.end()
        return image

    def calculate_pointer(self, x, y, theta, distance, length):
        """calculates a line in the direction of something"""
        xDistance = distance * math.sin(math.radians(theta + 90.0))
        yDistance = distance * math.cos(math.radians(theta + 90.0))
        xLength = xDistance + length*math.sin(math.radians(theta + 90.0))
        yLength = yDistance + length*math.cos(math.radians(theta + 90.0))
        return QtCore.QLineF(xDistance + x, yDistance + y,
                             xLength + x, yLength + y)

    def calculate_color(self, p, saturation, value):
        """determines the color of a particle"""
        # the color of one point is based on how many
        # other points are close to it, "close" here
        # being defined very arbitrarily
        near = 10 # pixels
        numClosePoints = len(filter(
          lambda q: near >= abs(q.x['display'] - p.x['display']) and
                    near >= abs(q.y['display'] - p.y['display']),
          self.particles))
        hue = 300 * (1 - numClosePoints/(self.numParticles*0.75))
        if hue < 0.0: hue = 0
        if hue > 300.0: hue = 300
        return QtGui.QColor.fromHsv(hue, saturation, value)

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

    def erase_mcl(self):
        self.particles = []
        Particle.selected = None

    def offset_slider_change(self):
        """Slot for position offset sliders"""
        sender = self.sender()
        if sender == self.x_offset_slider:
            D.robot.x['offset'] = self.x_offset_slider.value()
            self.x_offset_field.setText(str(D.robot.x['offset']))
        elif sender == self.y_offset_slider:
            D.robot.y['offset'] = self.y_offset_slider.value()
            self.y_offset_field.setText(str(D.robot.y['offset']))
        elif sender == self.theta_offset_slider:
            D.robot.t['offset'] = self.theta_offset_slider.value()
            self.theta_offset_field.setText(str(D.robot.t['offset']))
            D.robot.pivot = (D.robot.x['prev'], D.robot.y['prev'])
        self.erase_trail()
        self.erase_mcl()

    def offset_field_change(self):
        """Slot for position offset text fields"""
        sender = self.sender()
        if sender == self.x_offset_field:
            D.robot.x['offset'] = int(self.x_offset_field.text())
            self.x_offset_slider.blockSignals(True)
            self.x_offset_slider.setSliderPosition(D.robot.x['offset'])
            self.x_offset_slider.blockSignals(False)
        elif sender == self.y_offset_field:
            D.robot.y['offset'] = int(self.y_offset_field.text())
            self.y_offset_slider.blockSignals(True)
            self.y_offset_slider.setSliderPosition(D.robot.y['offset'])
            self.y_offset_slider.blockSignals(False)
        elif sender == self.theta_offset_field:
            D.robot.t['offset'] = int(self.theta_offset_field.text())
            self.theta_offset_slider.blockSignals(True)
            self.theta_offset_slider.setSliderPosition(D.robot.t['offset'])
            self.theta_offset_slider.blockSignals(False)
            D.robot.pivot = (D.robot.x['prev'], D.robot.y['prev'])
        self.erase_trail()
        self.erase_mcl()

    def image_scale_change(self):
        pass

    ###
    # Position slots
    ###
    def position_reset(self):
        """Position reset buttons connect to this"""
        sender = self.sender()
        if sender == self.x_reset_button or sender == self.all_reset_button:
            D.robot.x['diff'] = D.robot.x['raw']
        if sender == self.y_reset_button or sender == self.all_reset_button:
            D.robot.y['diff'] = D.robot.y['raw']
        if (sender == self.theta_reset_button or 
                sender == self.all_reset_button):
            D.robot.t['diff'] = D.robot.t['raw']
            D.robot.pivot = (D.robot.x['prev'], D.robot.y['prev'])
        self.erase_trail()
        self.erase_mcl()

    def virtual_control_toggle(self):
        """Connects to virtual control checkbox"""
        global D
        D.use_gui_control = (True if self.virtual_control_checkbox.isChecked()
          else False)
        self.virtual_group.setEnabled(D.use_gui_control)

    def virtual_robot_step_set(self):
        """Sets pixel move amount for virtual robot control"""
        sender = self.sender()
        if sender == self.step_meter_field:
            self.virtual_robot_meter_step = float(self.step_meter_field.text())
        elif sender == self.step_degree_field:
            self.virtual_robot_degree_step = float(
              self.step_degree_field.text())

    def virtual_move(self): # TODO: make it look nice
        """Moves robot by virtual_robot_step pixels"""
        sender = self.sender()
        Robot.recent_move = True
        if sender == self.forward_button:
            x_displacement = self.virtual_robot_meter_step * math.cos(
              math.radians(D.robot.t['display']))
            y_displacement = self.virtual_robot_meter_step * math.sin(
              math.radians(D.robot.t['display']))
            D.robot.x['prev'] = D.robot.x['raw']
            D.robot.y['prev'] = D.robot.y['raw']
            D.robot.x['raw'] += x_displacement
            D.robot.y['raw'] += y_displacement
            Robot.recent_move = "Forward"
        elif sender == self.backward_button:
            x_displacement = self.virtual_robot_meter_step * math.cos(
              math.radians(D.robot.t['display']))
            y_displacement = self.virtual_robot_meter_step * math.sin(
              math.radians(D.robot.t['display']))
            D.robot.x['prev'] = D.robot.x['raw']
            D.robot.y['prev'] = D.robot.y['raw']
            D.robot.x['raw'] -= x_displacement
            D.robot.y['raw'] -= y_displacement
            Robot.recent_move = "Backward"
        elif sender == self.turn_left_button:
            D.robot.t['raw'] += self.virtual_robot_degree_step
        elif sender == self.turn_right_button:
            D.robot.t['raw'] -= self.virtual_robot_degree_step
            print self.imageMap.white_lines, self.imageMap.black_lines
        D.robot.update_sensor_values(virtual=True, image_map=self.imageMap)
        

def sensor_callback( data ):
    """sensor_callback is called for each sensorPacket message"""
    global D
    if not D.use_gui_control:
        D.robot.charge_level = '{:.0%}'.format(data.chargeLevel)
        # data comes in as meters and radians
        D.robot.x['prev'] = D.robot.x['raw']
        D.robot.y['prev'] = D.robot.y['raw']
        D.robot.x['raw'] = data.x
        D.robot.y['raw'] = data.y
        D.robot.t['raw'] = math.degrees(data.theta)
        D.robot.update_sensor_values([data.cliffLeftSignal,
                                      data.cliffFrontLeftSignal,
                                      data.cliffFrontRightSignal,
                                      data.cliffRightSignal])
        Robot.recent_move = (True if D.robot.x['raw'] != D.robot.x['prev'] or
            D.robot.y['raw'] != D.robot.y['prev'] else False)


if __name__ == '__main__':
    import sys
    rospy.init_node('RobotBox')

    # set up a callback for the sensorPacket stream, i.e., "topic"
    rospy.Subscriber( 'sensorPacket', SensorPacket, sensor_callback )

    app = QtGui.QApplication(sys.argv)
    display = RobotGUI()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
