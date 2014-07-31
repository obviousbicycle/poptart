#!/usr/bin/env python

# A window displaying the Create's movements based on its odometry

# General mathy stuff
import random
import time
import math

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
USE_CM = True # Meters or centimeters
SCALE = 1.0 # How many distance units for one pixel? TODO
WIDGET_SPACING = 10 # Pixels of space between widgets
DEFAULT_SIZE = 300 # side length of default image


class Data: pass    # Empty class for a generic data holder
D = Data()
D.use_gui_control = False


class Sensor(object):

    """Represents IR light sensor on a robot.

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
    x, y -- a dictionary of the robot's coordinates
        'raw' is the value coming directly from the robot
        'display' is what is shown on the GUI
        'draw' is passed into graphics-drawing methods that set (0, 0)
            at the upper-left corner
        'offset' stores values from offset sliders
        'diff' is used with the reset buttons
        'prev' is the previous raw value
    t -- a dictionary of theta values, theta being the angle at which
        the robot would move forward, relative to the horizontal and
        increasing in the counterclockwise direction
    pivot -- a tuple (x, y, t) used for correctly drawing the robot
    sensors -- an array of four Sensor objects, arranged as such:
        [back left, front left, front right, back right]
    charge_level

    Class variables:
    sensor_angles -- measurements from actual robot
    recent_move

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
    recent_move = False

    def __init__(self):
        """Initialize robot with values"""
        self.x = {'raw': 0.0, 'display': 0.0, 'draw': 0.0, 'offset': 0,
                  'diff': 0.0, 'prev': 0.0}
        self.y = {'raw': 0.0, 'display': 0.0, 'draw': 0.0, 'offset': 0,
                  'diff': 0.0, 'prev': 0.0}
        self.t = {'raw': 0.0, 'display': 0.0, 'offset': 0, 'diff': 0.0}
        self.pivot = ()
        # The Cartesian coordinates of the sensors when theta = 0 deg
        self.sensors = [Sensor(6.71, 15.07), Sensor(15.86, 4.55),
                        Sensor(-15.86, -4.55), Sensor(-6.71, -15.07)]
        self.charge_level = ""

    def update_display(self):
        """Updates the coordinates displayed in the "Position" tab,
        which are also the Cartesian coordinates of the robot in the
        display image
        """
        theta_display = self.t['offset'] + self.t['raw'] - self.t['diff']
        while theta_display > 359: theta_display -= 360
        while theta_display < 0: theta_display += 360
        if self.t['diff'] or self.t['offset']:
            # A set pivot was used to get the robot marker to move in
            # the proper direction after changing theta offset/diff.
            # Changing offset/diff more than once may cause the robot
            # marker to teleport weirdly.
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
        self.x['display'] = (100.0**USE_CM*(x_actual-self.x['diff']) +
          self.x['offset']/SCALE)
        self.y['display'] = (100.0**USE_CM*(y_actual-self.y['diff']) +
          self.y['offset']/SCALE)
        self.t['display'] = theta_display
        
    def update_draw(self, w, h):
        """Updates x/y draw values"""
        origin = (w/2, h/2)
        self.x['draw'] = origin[0] + self.x['display']*SCALE
        self.y['draw'] = origin[1] - self.y['display']*SCALE

    def update_sensor_positions(self):
        """Update sensor coordinates using the robot's coordinates"""
        for s in self.sensors:
            distance = math.hypot(s.x, s.y)
            angle = (self.sensor_angles[self.sensors.index(s)] - 
              self.t['display'])
            s.x = math.cos(math.radians(angle)) * distance
            s.y = math.sin(math.radians(angle)) * distance

    def update_sensor_values(self, values=[], virtual=False,
                             image_map=QtGui.QImage()):
        """Update sensor light values"""
        if values: # True only when robot is connected
            for i in xrange(4):
                self.sensors[i].light = values[i]
        elif image_map.isNull():
            for s in self.sensors:
                s.light = -1
        elif virtual:
            sensor_points = self.get_sensor_draw_position()
            for i in xrange(4):
                if (sensor_points[i].x >= 0 and
                        sensor_points[i].x <= image_map.width() and
                        sensor_points[i].y >= 0 and
                        sensor_points[i].y <= image_map.height()):
                    # sensors get assigned color value depending on
                    # color of pixel underneath on map
                    color_at_point = QtGui.QColor.fromRgb(
                      image_map.pixel(sensor_points[i].x, sensor_points[i].y))
                    if color_at_point == QtGui.QColor("white"):
                        self.sensors[i].light = 1000
                    elif color_at_point == QtGui.QColor("black"):
                        self.sensors[i].light = 0
                    else:
                        self.sensors[i].light = 200
                else:
                    # Sensors out of image bounds get invalid value
                    self.sensors[i].light = -1

    def get_sensor_global_position(self):
        """Use the robot's x, y, theta to convert its Sensor object
        coordinates to global coordinates
        """
        self.update_sensor_positions()
        return [Sensor(s.x+self.x['display'], s.y+self.y['display'], s.light)
                for s in self.sensors]

    def get_sensor_draw_position(self):
        """Use the robot's x, y, theta to convert its Sensor object
        coordinates to equivalent coordinates for drawing graphics
        """
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
    update_display -- 
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
        self.sensors = [Sensor(6.71, 15.07), Sensor(15.86, 4.55),
                        Sensor(-15.86, -4.55), Sensor(-6.71, -15.07)]
        self.prob = probability

    def update_display(self, x, y, t, direction):
        """Moves the particle as specified by the arguments"""
        magnitude = math.hypot(x, y)
        self.t['display'] += t
        while self.t['display'] > 359: self.t['display'] -= 360
        while self.t['display'] < 0: self.t['display'] += 360
        if not D.use_gui_control: # TODO: fix marker moving backward when robot moves forward
            direction = "Forward"
        x = math.cos(math.radians(self.t['display'])) * magnitude
        y = math.sin(math.radians(self.t['display'])) * magnitude
        if direction == "Forward":
            self.x['display'] += x
            self.y['display'] += y
        if direction == "Backward":
            self.x['display'] -= x
            self.y['display'] -= y

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
    image_map -- image map of area
    make_trail -- toggle robot trail
    make_mcl -- toggle MCL
    x_offset, y_offset, t_offset -- robot marker offset in pixels
    trail -- list of all past robot locations, from oldest to newest
    recent_move -- true if robot moved since last GUI update
    x_diff, y_diff, t_diff -- for "clearing" odometer
    pivot -- for theta offset/reset purposes
    thresholds -- user-input light thresholds
    num_particles -- how many MCL particles
    particles -- list of Particle objects
    xy_noise, t_noise -- MCL resampled points noise, as sigma of
        Gaussian distribution
    status_message -- the message in the window's bottom status bar
    redraw_timer -- redraws GUI with each cycle
    """

    def __init__(self):
        # Construct the parent class
        super(RobotGUI, self).__init__()
        # Initialize data values related to visuals/drawing
        self.width = DEFAULT_SIZE
        self.height = DEFAULT_SIZE
        self.virtual_robot_meter_step = 0.01
        self.virtual_robot_degree_step = 1.0
        self.image_map = QtGui.QImage()
        self.make_trail = 0
        self.make_mcl = 0
        # Initialize array of past locations
        self.trail = []
        # Initialize light threshold array
        # First four are lower, last four are upper
        self.thresholds = [100, 180, 180, 150, 800, 1000, 1000, 1000]
        # Initialize data values related to MCL
        #self.num_particles = int(self.width * self.height * 0.01)
        self.num_particles = 1
        self.particles = []
        self.particle_radius = 0
        self.xy_noise = 0.0
        self.t_noise = 0.0
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
        self.save_action = QtGui.QAction("&Save image", self)
        self.save_action.setShortcut(QtGui.QKeySequence("Ctrl+S"))
        self.save_action.triggered.connect(self.save_image)
        self.open_action = QtGui.QAction("&Open map", self)
        self.open_action.triggered.connect(self.open_image)
        self.open_action.setShortcut(QtGui.QKeySequence("Ctrl+O"))
        self.clear_action = QtGui.QAction("Clear image && map", self)
        self.clear_action.triggered.connect(self.clear_image)
        file_menu = self.menuBar().addMenu("File")
        file_menu.addAction(self.save_action)
        file_menu.addAction(self.open_action)
        file_menu.addAction(self.clear_action)

    def init_central(self):
        """Displays top-down map over the robot"""
        # Setup our very basic GUI - a label which fills the whole
        # window and holds our image
        self.setWindowTitle('RobotBox')
        self.image_box = QtGui.QLabel(self)
        self.setCentralWidget(self.image_box)
        # Holds the status message to be displayed on the next
        # GUI update
        self.status_message = ''
        # A timer to redraw the GUI
        self.redraw_timer = QtCore.QTimer(self)
        self.redraw_timer.timeout.connect(self.redraw_callback)
        self.redraw_timer.start(GUI_UPDATE_PERIOD)

    def init_left(self):
        """Displays position data from sensorPacket, displays IR
        light sensor data from sensorPacket, and displays MCL
        particle info
        """
        leftTabs = QtGui.QTabWidget(self)

        # Tab for displaying position info and virtual control buttons
        positionGroup = QtGui.QWidget(self)
        # Coordinate display
        xLabel = QtGui.QLabel("X (" + USE_CM*"c" + "m): ", self)
        self.xValue = QtGui.QLabel(self)
        yLabel = QtGui.QLabel("Y (" + USE_CM*"c" + "m): ", self)
        self.yValue = QtGui.QLabel(self)
        thetaLabel = QtGui.QLabel("Theta (deg): ", self)
        self.thetaValue = QtGui.QLabel(self)
        # Coordinate reset buttons
        self.x_reset_button = QtGui.QPushButton("Reset", self)
        self.x_reset_button.clicked.connect(self.position_reset)
        self.y_reset_button = QtGui.QPushButton("Reset", self)
        self.y_reset_button.clicked.connect(self.position_reset)
        self.theta_reset_button = QtGui.QPushButton("Reset", self)
        self.theta_reset_button.clicked.connect(self.position_reset)
        self.all_reset_button = QtGui.QPushButton("Reset all", self)
        self.all_reset_button.clicked.connect(self.position_reset)
        # Virtual control buttons
        self.virtual_control_checkbox = QtGui.QCheckBox(
          "Use GUI/keyboard input as robot motion data", self)
        self.virtual_control_checkbox.toggled.connect(
          self.virtual_control_toggle)
        # Virtual control movement interval text fields
        self.step_label = QtGui.QLabel("Step: ", self)
        self.step_meter_label = QtGui.QLabel("m", self)
        self.step_degree_label = QtGui.QLabel("deg", self)
        meter_validator = QtGui.QDoubleValidator(
          1.0/(100.0**USE_CM), 50.0/(100.0**USE_CM), 2, self)
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
        # The control buttons themselves
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
        # Assembling virtual control subsublayout
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
        virtualLayout.setColumnStretch(0, 1)
        virtualLayout.setColumnStretch(20, 1)
        # Assembling position info sublayout
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
            (self.virtual_control_checkbox, 7, 1, 1, 5),
            (self.virtual_group, 8, 1, 1, 5)
        ]
        for args in physicalWidgetsToAdd: physicalLayout.addWidget(*args)
        physicalLayout.setColumnMinimumWidth(3, WIDGET_SPACING)
        physicalLayout.setRowMinimumHeight(6, WIDGET_SPACING)
        physicalLayout.setColumnStretch(0, 1)
        physicalLayout.setColumnStretch(6, 1)
        physicalLayout.addWidget
        # Assembling layout for positionGroup
        positionLayout = QtGui.QGridLayout()
        positionGroup.setLayout(physicalLayout)
        # Virtual control initially set to off
        self.virtual_control_toggle()

        # IR light sensor info tab
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
        light_fields = [
            self.backLeftLowerField, self.frontLeftLowerField,
            self.frontRightLowerField, self.backRightLowerField,
            self.backLeftUpperField, self.frontLeftUpperField,
            self.frontRightUpperField, self.backRightUpperField
        ]
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
        lightLayout.setRowStretch(10, 1)
        lightLayout.setColumnMinimumWidth(2, WIDGET_SPACING)
        lightLayout.setColumnMinimumWidth(4, WIDGET_SPACING)
        lightLayout.setColumnMinimumWidth(6, WIDGET_SPACING) 
        lightLayout.setColumnMinimumWidth(8, WIDGET_SPACING)

        # Info tab for selected MCL particle
        particle_group = QtGui.QWidget(self)
        # Labels for uninteresting MCL display settings
        self.off_label = QtGui.QLabel("MCL is currently turned off.", self)
        self.off_label.setAlignment(QtCore.Qt.AlignCenter)
        self.hide_label = QtGui.QLabel("MCL is currently hidden.", self)
        self.hide_label.setAlignment(QtCore.Qt.AlignCenter)
        self.show_widget = QtGui.QWidget()
        show_label = QtGui.QLabel(
          "Double click a particle for more info.", self)
        show_label.setAlignment(QtCore.Qt.AlignHCenter)
        # Location info of selected particle
        particle_info_group = QtGui.QGroupBox("Particle")
        particle_position_label = QtGui.QLabel("Position", self)
        self.particle_position_value = QtGui.QLabel(self)
        particle_theta_label = QtGui.QLabel("Theta")
        self.particle_theta_value = QtGui.QLabel(self)
        particle_weight_label = QtGui.QLabel("Weight", self)
        self.particle_weight_value = QtGui.QLabel(self)
        # Assembling position info subsublayout
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
        # Sensor info of selected particle
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
        # Assembling sensor info subsublayout
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
        # Assembling selected particle info sublayout
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

        # Putting everything together
        leftTabs.addTab(positionGroup, "Position")
        leftTabs.addTab(lightGroup, "IR sensors")
        leftTabs.addTab(particle_group, "Particles")
        leftTabs.setMinimumWidth(450)
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

        # Image display settings tab
        displayGroup = QtGui.QWidget(self)
        self.robotMarkerCheckbox = QtGui.QCheckBox("Robot marker", self)
        self.robotMarkerCheckbox.setChecked(True)
        self.axesCheckbox = QtGui.QCheckBox("Axes", self)
        self.gridCheckbox = QtGui.QCheckBox("Grid", self)
        # Trail draw settings
        trailLabel = QtGui.QLabel("Robot trail", self)
        self.showTrailButton = QtGui.QRadioButton("Show", self)
        self.showTrailButton.clicked.connect(self.set_trail)
        self.hideTrailButton = QtGui.QRadioButton("Hide", self)
        self.hideTrailButton.clicked.connect(self.set_trail)
        self.noTrailButton = QtGui.QRadioButton("Off", self)
        self.noTrailButton.clicked.connect(self.set_trail)
        self.eraseTrailButton = QtGui.QPushButton("Erase", self)
        self.eraseTrailButton.clicked.connect(self.erase_trail)
        # Limit radio button choices to trail-related buttons
        trailGroup = QtGui.QButtonGroup(self)
        trailGroup.addButton(self.showTrailButton)
        trailGroup.addButton(self.hideTrailButton)
        trailGroup.addButton(self.noTrailButton)
        # MCL draw settings
        MCLLabel = QtGui.QLabel("Monte Carlo", self)
        self.showMCLButton = QtGui.QRadioButton("Show", self)
        self.showMCLButton.clicked.connect(self.set_mcl)
        self.hideMCLButton = QtGui.QRadioButton("Hide", self)
        self.hideMCLButton.clicked.connect(self.set_mcl)
        self.noMCLButton = QtGui.QRadioButton("Off", self)
        self.noMCLButton.clicked.connect(self.set_mcl)
        self.eraseMCLButton = QtGui.QPushButton("Erase", self)
        self.eraseMCLButton.clicked.connect(self.erase_mcl)
        # Limit radio button choices to MCL-related buttons
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
        displayLayout.setRowMinimumHeight(2, WIDGET_SPACING)
        displayLayout.setRowMinimumHeight(6, WIDGET_SPACING)
        displayLayout.setRowStretch(20, 1)
        # Set initial button states
        self.noTrailButton.setChecked(True)
        self.noMCLButton.setChecked(True)
        self.eraseTrailButton.setEnabled(False)
        self.eraseMCLButton.setEnabled(False)

        # Robot marker offset tab
        offsetGroup = QtGui.QWidget(self)
        # X offset text field
        x_offsetLabel = QtGui.QLabel("X", self)
        self.x_offset_field = QtGui.QLineEdit("0")
        self.x_offset_field.editingFinished.connect(self.offset_field_change)
        x_offsetValidator = QtGui.QIntValidator(
          -self.width/2, self.width/2, self)
        self.x_offset_field.setValidator(x_offsetValidator)
        # X offset slider
        self.x_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.x_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.x_offset_slider.setMinimum(-self.width)
        self.x_offset_slider.setMaximum(self.width)
        self.x_offset_slider.setTickInterval(self.width/10)
        self.x_offset_slider.setValue(0)
        self.x_offset_slider.valueChanged.connect(self.offset_slider_change)
        # Y offset text field
        y_offsetLabel = QtGui.QLabel("Y", self)
        self.y_offset_field = QtGui.QLineEdit("0")
        self.y_offset_field.editingFinished.connect(self.offset_field_change)
        y_offsetValidator = QtGui.QIntValidator(
          -self.height / 2, self.height / 2, self)
        self.y_offset_field.setValidator(y_offsetValidator)
        # Y offset slider
        self.y_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.y_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.y_offset_slider.setMinimum(-self.height)
        self.y_offset_slider.setMaximum(self.height)
        self.y_offset_slider.setTickInterval(self.height/10)
        self.y_offset_slider.setValue(0)
        self.y_offset_slider.valueChanged.connect(self.offset_slider_change)
        # Theta offset text field
        t_offsetLabel = QtGui.QLabel("Theta", self)
        self.t_offset_field = QtGui.QLineEdit("0")
        self.t_offset_field.editingFinished.connect(
          self.offset_field_change)
        t_offsetValidator = QtGui.QIntValidator(0, 359, self)
        self.t_offset_field.setValidator(t_offsetValidator)
        # Theta offset slider
        self.t_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.t_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.t_offset_slider.setMinimum(0)
        self.t_offset_slider.setMaximum(359)
        self.t_offset_slider.setTickInterval(15)
        self.t_offset_slider.setValue(0)
        self.t_offset_slider.valueChanged.connect(
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
                       (t_offsetLabel, 7, 0),
                 (self.t_offset_field, 7, 1),
                (self.t_offset_slider, 8, 0, 1, 2)
        ]
        for args in offsetWidgetsToAdd: offsetLayout.addWidget(*args)
        offsetLayout.setColumnStretch(0, 1)
        offsetLayout.setRowStretch(20, 1)
        
        # MCL tab
        mclGroup = QtGui.QWidget(self)
        num_particlesLabel = QtGui.QLabel("# particles", self)
        # Number of MCL particles nput field
        self.num_particlesField = QtGui.QLineEdit(str(self.num_particles))
        self.num_particlesField.editingFinished.connect(self.num_particles_set)
        num_particlesMax = self.width*self.height
        num_particlesValidator = QtGui.QIntValidator(
          1, num_particlesMax, self)
        self.num_particlesField.setValidator(num_particlesValidator)
        self.num_particlesField.setMaxLength(len(str(num_particlesMax)))
        # XY noise text field
        # Apparently sliders only accept integer values??
        xy_noiseLabel = QtGui.QLabel("XY noise", self)
        self.xy_noiseField = QtGui.QLineEdit(str(self.xy_noise))
        self.xy_noiseField.editingFinished.connect(self.noise_field_change)
        xy_noiseMax = 10.0
        xy_noiseValidator = QtGui.QDoubleValidator(
          0.0, xy_noiseMax, 1, self)
        self.xy_noiseField.setValidator(xy_noiseValidator)
        self.xy_noiseField.setMaxLength(len(str(xy_noiseMax)))
        # XY noise slider
        self.xy_noiseSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.xy_noiseSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.xy_noiseSlider.setMinimum(0)
        self.xy_noiseSlider.setMaximum(10*xy_noiseMax)
        self.xy_noiseSlider.setTickInterval(5)
        self.xy_noiseSlider.setValue(10*self.xy_noise)
        self.xy_noiseSlider.valueChanged.connect(self.noise_slider_change)
        # Theta noise text field
        t_noiseLabel = QtGui.QLabel("Theta noise", self)
        self.t_noiseField = QtGui.QLineEdit(str(self.t_noise))
        self.t_noiseField.editingFinished.connect(self.noise_field_change)
        t_noiseMax = 5.0
        t_noiseValidator = QtGui.QDoubleValidator(
          0.0, t_noiseMax, 1, self)
        self.t_noiseField.setValidator(t_noiseValidator)
        self.t_noiseField.setMaxLength(len(str(t_noiseMax)))
        # Theta noise slider
        self.t_noiseSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.t_noiseSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.t_noiseSlider.setMinimum(0)
        self.t_noiseSlider.setMaximum(10*t_noiseMax)
        self.t_noiseSlider.setTickInterval(5)
        self.t_noiseSlider.setValue(10*self.t_noise)
        self.t_noiseSlider.valueChanged.connect(self.noise_slider_change)
        # Checkboxes for particle draw settings
        self.particleDetailCheckbox = QtGui.QCheckBox(
          "Detailed particles", self)
        self.particleColoringCheckbox = QtGui.QCheckBox(
          "Variable particle colors", self)
        # mclGroup layout
        mclLayout = QtGui.QGridLayout()
        mclGroup.setLayout(mclLayout)
        mclWidgetsToAdd = [
                       (num_particlesLabel, 3, 0),
                  (self.num_particlesField, 3, 1),
                            (xy_noiseLabel, 5, 0),
                       (self.xy_noiseField, 5, 1),
                      (self.xy_noiseSlider, 6, 0, 1, 2),
                             (t_noiseLabel, 7, 0),
                        (self.t_noiseField, 7, 1),
                       (self.t_noiseSlider, 8, 0, 1, 2),
              (self.particleDetailCheckbox, 9, 0, 1, 2),
            (self.particleColoringCheckbox, 10, 0, 1, 2)
        ]
        for args in mclWidgetsToAdd: mclLayout.addWidget(*args)
        mclLayout.setColumnStretch(0, 1)
        mclLayout.setRowStretch(20, 1)
        # Initial checkboxes settings
        self.particleDetailCheckbox.setChecked(True)
        self.particleColoringCheckbox.setChecked(True)

        # Putting everything together
        rightTabs.addTab(displayGroup, "Draw")
        rightTabs.addTab(offsetGroup, "Offset")
        rightTabs.addTab(mclGroup, "MCL")
        self.rightDock = QtGui.QDockWidget("", self)
        self.rightDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        self.rightDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.rightDock.setTitleBarWidget(QtGui.QWidget(self))
        self.rightDock.setWidget(rightTabs)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.rightDock)

    def keyPressEvent(self, event):
        """Reimplements PySide's keyPressEvent to allow arrow key
        control of the robot marker"""
        # Must click on window before control is enabled
        if self.hasFocus() and self.virtual_control_checkbox.isChecked():
            Robot.recent_move = True
            if event.key() == QtCore.Qt.Key_Up:
                x_displacement = self.virtual_robot_meter_step * math.cos(
                  math.radians(D.robot.t['display']))
                y_displacement = self.virtual_robot_meter_step * math.sin(
                  math.radians(D.robot.t['display']))
                D.robot.x['raw'] += x_displacement
                D.robot.y['raw'] += y_displacement
                Robot.recent_move = "Forward"
            elif event.key() == QtCore.Qt.Key_Down:
                x_displacement = self.virtual_robot_meter_step * math.cos(
                  math.radians(D.robot.t['display']))
                y_displacement = self.virtual_robot_meter_step * math.sin(
                  math.radians(D.robot.t['display']))
                D.robot.x['raw'] -= x_displacement
                D.robot.y['raw'] -= y_displacement
                Robot.recent_move = "Backward"
            elif event.key() == QtCore.Qt.Key_Left:
                D.robot.t['raw'] += self.virtual_robot_degree_step
            elif event.key() == QtCore.Qt.Key_Right:
                D.robot.t['raw'] -= self.virtual_robot_degree_step
            
    def mouseDoubleClickEvent(self, event):
        """Can double click on MCL particles to show info"""
        if self.make_mcl == 2 and self.image_box == self.childAt(event.pos()):
            # Reported click location is relative to the entire window,
            # so we have to adjust using the sizes of the other docks
            x_offset = self.leftDock.width() + 6
            y_offset = (max(self.leftDock.height(), self.rightDock.height()) -
              self.height)/2
            relative_click_pos = (event.x() - x_offset, event.y() - y_offset)
            closest_distance = self.particle_radius + 1
            for p in self.particles:
                distance = math.hypot(
                  relative_click_pos[0] - p.x['draw'],
                  relative_click_pos[1] - p.y['draw'])
                if distance < closest_distance:
                    # The closest particle to the double-click location
                    # will be selected
                    closest_distance = distance
                    Particle.selected = p
            if closest_distance == self.particle_radius + 1:
                Particle.selected = None

    def redraw_callback(self):
        """The main GUI update method"""
        global D
        # Update the background image
        image = QtGui.QPixmap()
        if self.image_map:
            image.convertFromImage(self.image_map)
        else:
            image.convertFromImage(QtGui.QImage(
              self.width,self.height,QtGui.QImage.Format_RGB888))
            image.fill(QtGui.QColor.fromHsv(240, 63, 127))

        # Update robot display and info
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

        # Set sensor info display
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

        if self.make_mcl:
           self.mcl_update()
        self.particle_info_update()

        if (not self.statusBar().currentMessage() or 
                "Charge" in self.statusBar().currentMessage()):
            self.statusBar().showMessage("Charge: " + D.robot.charge_level)

        image = self.display_update(image)
        self.resize(self.width, self.height)
        self.image_box.setPixmap(image)

    def mcl_update(self):
        """The MCL algorithm"""
        if not self.particles:
            self.particles = [Particle(
                random.uniform(-self.width/2, self.width/2),
                random.uniform(-self.height/2, self.height/2),
                random.uniform(0, 360),
                1.0 / self.num_particles)
              for i in xrange(self.num_particles)]
        else:
            for p in self.particles:
                p.update_draw(self.width, self.height)
                p.update_sensor_positions()
                p.update_sensor_values(virtual=True, image_map=self.image_map)
        if Robot.recent_move:
            # Only do MCL things when we move
            for p in self.particles:
                # Motion update
                p.update_display(self.trail[-1][0]-self.trail[-2][0],
                                 self.trail[-1][1]-self.trail[-2][1], 
                                 self.trail[-1][2]-self.trail[-2][2],
                                 Robot.recent_move)
                if (abs(p.x['display']) > self.width/2 or
                        abs(p.y['display']) > self.height/2):
                    p.prob *= 0.01
                # Sensor update
                # light_difference = max(map(lambda observed, expected: abs(observed.light-expected.light), D.robot.sensors, p.sensors))
                # if light_difference > 0:
                #     p.prob *= 1.0/light_difference
            Robot.recent_move = False
            oldSumProb = math.fsum([p.prob for p in self.particles])
            if oldSumProb <= 0.01:
                # If all the points are really unlikely, just start over
                self.particles = []
            else: # TODO
                # Resampling creates an entirely new list of particles
                # based on the probabilities from the old generation
                newGen = []
                probs = [p.prob for p in self.particles]
                cumulativeProb = [math.fsum(probs[i::-1]) for i in xrange(
                  self.num_particles)]
                counter = 0
                for i in xrange(self.num_particles):
                    # Step through probability "blocks" of particle list
                    newGen.append(self.particles[counter])
                    while (i*oldSumProb/self.num_particles >
                            cumulativeProb[counter]):
                        counter += 1
                newSumProb = math.fsum([p.prob for p in newGen])
                self.particles = newGen
                for p in self.particles:
                    # Resampled particles' probabilities normalized
                    p.finish_resample(
                      self.xy_noise, self.t_noise, newSumProb)
        
    def particle_info_update(self):
        """Updates particle info widget box"""
        if self.make_mcl == 0:
            self.particle_layout.setCurrentWidget(self.off_label)
        elif self.make_mcl == 1:
            self.particle_layout.setCurrentWidget(self.hide_label)
        elif self.make_mcl == 2:
            if Particle.selected is not None:
                self.particle_position_value.setText(
                  '({:.2f}, {:.2f})'.format(
                    Particle.selected.x['display'],
                    Particle.selected.y['display']))
                self.particle_theta_value.setText(
                  '{:.1f}'.format(Particle.selected.t['display']))
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
                for f in to_clear: f("")
            self.particle_layout.setCurrentWidget(self.show_widget)

    def display_update(self, image):
        """Updates display image"""
        painter = QtGui.QPainter()
        painter.begin(image)
        # Drawing axes
        if self.axesCheckbox.isChecked():
            painter.setPen(QtGui.QColor.fromHsv(240, 63, 63))
            painter.setBrush(QtGui.QColor.fromHsv(240, 63, 63))
            painter.drawLine(self.width/2, 0, self.width/2, self.height)
            painter.drawLine(0, self.height/2, self.width, self.height/2)
        # Drawing grid
        if self.gridCheckbox.isChecked():
            spacing = 20
            painter.setPen(QtGui.QColor.fromHsv(240,63,63))
            painter.setBrush(QtGui.QColor.fromHsv(240,63,63))
            # grid is centered on origin
            for x in xrange((self.width/2)%spacing, self.width, spacing):
                for y in xrange((self.height/2)%spacing, self.height, spacing):
                    painter.drawEllipse(x, y, 1, 1)
        # Drawing particles
        if self.make_mcl == 2:
            self.particle_radius = (3 if self.particleDetailCheckbox.isChecked()
              else 2)
            selected_radius = self.particle_radius + 2
            get_color = (self.calculate_color if
                self.particleColoringCheckbox.isChecked() else
                lambda x, y, z: QtGui.QColor("darkGray"))
            for p in self.particles:
                color = get_color(p, 255, 180)
                painter.setPen(color)
                painter.setBrush(color)
                painter.drawEllipse(QtCore.QPoint(p.x['draw'], p.y['draw']), 
                  self.particle_radius, self.particle_radius)
            if self.particleDetailCheckbox.isChecked():
                # Draw particle sensors and particle heading
                for p in self.particles:
                    painter.setPen("black")
                    sensorDraw = p.get_sensor_draw_position()
                    for s in sensorDraw:
                        painter.drawPoint(s.x, s.y)
                    painter.drawLine(self.calculate_pointer(
                      p.x['draw'], p.y['draw'], p.t['display'],
                      self.particle_radius + 3, 4))
            if Particle.selected is not None:
                # Draw the selected particle
                color = self.calculate_color(Particle.selected, 255, 255)
                painter.setPen("black")
                painter.setBrush(color)
                painter.drawEllipse(
                  QtCore.QPoint(
                    Particle.selected.x['draw'], Particle.selected.y['draw']), 
                  selected_radius, selected_radius)
                # Sensors
                painter.setPen("white")
                sensorDraw = Particle.selected.get_sensor_draw_position()
                for s in sensorDraw:
                    painter.drawPoint(s.x, s.y)
                # Pointer
                painter.drawLine(self.calculate_pointer(
                  Particle.selected.x['draw'], Particle.selected.y['draw'],
                  Particle.selected.t['display'], self.particle_radius + 5, 6))
        # Drawing robot trail
        if self.make_trail == 2 and len(self.trail) >= 2:
            painter.setPen(QtGui.QColor(255, 0, 0))
            painter.setBrush(QtGui.QColor(255, 0, 0))
            for p in xrange(1,len(self.trail)):
                painter.drawLine(self.trail[p-1][0], self.trail[p-1][1],
                                 self.trail[p][0], self.trail[p][1])
        # Drawing robot marker
        if self.robotMarkerCheckbox.isChecked():
            markerRadius = 8
            painter.setPen(QtGui.QColor("black"))
            painter.setBrush(QtGui.QColor("lightGray"))
            painter.drawEllipse(
              QtCore.QPoint(self.trail[-1][0], self.trail[-1][1]),
              markerRadius, markerRadius)
            # Sensors
            painter.setPen(QtGui.QColor(0,255,0))
            robot_sensor_draw = D.robot.get_sensor_draw_position()
            for s in robot_sensor_draw:
                painter.drawPoint(s.x, s.y)
            # Pointer
            painter.drawLine(self.calculate_pointer(
              self.trail[-1][0], self.trail[-1][1],
              float(self.thetaValue.text()), markerRadius + 3, 8))
        painter.end()
        return image

    def calculate_pointer(self, x, y, theta, distance, length):
        """Calculates a line with the given length and an angle of
        theta degrees, offset from (x, y) by distance
        """
        xDistance = distance * math.sin(math.radians(theta + 90.0))
        yDistance = distance * math.cos(math.radians(theta + 90.0))
        xLength = xDistance + length*math.sin(math.radians(theta + 90.0))
        yLength = yDistance + length*math.cos(math.radians(theta + 90.0))
        return QtCore.QLineF(xDistance + x, yDistance + y,
                             xLength + x, yLength + y)

    def calculate_color(self, p, saturation, value):
        """Determines the color of a particle"""
        # The color of one point is based on how many other points
        # are close to it, "close" here being defined very arbitrarily
        near = 10 # pixels
        numClosePoints = len(filter(
          lambda q: near >= abs(q.x['display'] - p.x['display']) and
                    near >= abs(q.y['display'] - p.y['display']),
          self.particles))
        hue = 300 * (1 - numClosePoints/(self.num_particles*0.75))
        if hue < 0.0: hue = 0
        if hue > 300.0: hue = 300
        return QtGui.QColor.fromHsv(hue, saturation, value)

    ###
    # MCL parameter slots
    ###
    def noise_slider_change(self):
        """Called by self.xy_noiseSlider and self.t_noiseSlider to
        change noise parameters and text fields
        """
        sender = self.sender()
        if sender == self.xy_noiseSlider:
            self.xy_noise = self.xy_noiseSlider.value() / 10.0
            self.xy_noiseField.setText(str(self.xy_noise))
        elif sender == self.t_noiseSlider:
            self.t_noise = self.t_noiseSlider.value() / 10.0
            self.t_noiseField.setText(str(self.t_noise))

    def noise_field_change(self):
        """Called by self.xy_noiseField and self.t_noiseField to
        change noise parameters and sliders
        """
        sender = self.sender()
        if sender == self.xy_noiseField:
            self.xy_noise = float(self.xy_noiseField.text())
            self.xy_noiseSlider.blockSignals(True)
            self.xy_noiseSlider.setSliderPosition(10 * self.xy_noise)
            self.xy_noiseSlider.blockSignals(False)
        elif sender == self.t_noiseField:
            self.t_noise = float(self.t_noiseField.text())
            self.t_noiseSlider.blockSignals(True)
            self.t_noiseSlider.setSliderPosition(10 * self.t_noise)
            self.t_noiseSlider.blockSignals(False)

    def num_particles_set(self):
        """Called by num_particlesField to change number of MCL
        particles
        """
        self.num_particles = int(self.num_particlesField.text())

    ###
    # Light sensor slot
    ###
    def light_threshold_set(self):
        """Called by light threshold field widgets to change light
        thresholds
        """
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
        """ Called by self.save_action to open save dialog"""
        dialog = QtGui.QFileDialog(self)
        dialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
        dialog.setViewMode(QtGui.QFileDialog.Detail)
        fname, _ = dialog.getSaveFileName(
          self, "Save image", "/home/robotics/Desktop/", "*.png")
        saving = self.image_box.pixmap().toImage()
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
        """ Called by self.open_action to open load dialog"""
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
            self.image_map = toLoad
            self.setWindowTitle('RobotBox - ' + fname)
            self.erase_trail()
            self.erase_mcl()
            self.statusBar().showMessage("Image " + fname + " opened", 3000)
        else:
            self.statusBar().showMessage("Failed to open image", 3000)
    
    def clear_image(self):
        """ Called by self.clear_action to clear current image map"""
        self.width = DEFAULT_SIZE
        self.height = DEFAULT_SIZE
        self.image_map = QtGui.QImage()
        self.setWindowTitle('RobotBox')
        self.erase_trail()
        self.erase_mcl()
        self.statusBar().showMessage("Cleared image", 3000)

    ###
    # Drawing slots
    ###
    def set_trail(self):
        """Called by buttons that change trail display settings"""
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
        """Called by buttons that change MCL display settings"""
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

    def erase_mcl(self): self.particles = []; Particle.selected = None

    def offset_slider_change(self):
        """Called by offset sliders to change robot marker offset
        parameters and text fields
        """
        sender = self.sender()
        if sender == self.x_offset_slider:
            D.robot.x['offset'] = self.x_offset_slider.value()
            self.x_offset_field.setText(str(D.robot.x['offset']))
        elif sender == self.y_offset_slider:
            D.robot.y['offset'] = self.y_offset_slider.value()
            self.y_offset_field.setText(str(D.robot.y['offset']))
        elif sender == self.t_offset_slider:
            D.robot.t['offset'] = self.t_offset_slider.value()
            self.t_offset_field.setText(str(D.robot.t['offset']))
            D.robot.pivot = (D.robot.x['prev'], D.robot.y['prev'])
        self.erase_trail()
        self.erase_mcl()

    def offset_field_change(self):
        """Called by offset text fields to change robot marker offset
        parameters and sliders
        """
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
        elif sender == self.t_offset_field:
            D.robot.t['offset'] = int(self.t_offset_field.text())
            self.t_offset_slider.blockSignals(True)
            self.t_offset_slider.setSliderPosition(D.robot.t['offset'])
            self.t_offset_slider.blockSignals(False)
            D.robot.pivot = (D.robot.x['prev'], D.robot.y['prev'])
        self.erase_trail()
        self.erase_mcl()

    ###
    # Position slots
    ###
    def position_reset(self):
        """Called by reset buttons to set diff value to the value the
        robot is currently reporting. The diff value is subtracted from
        the display value to give the appearance of resetting the
        robot's position.
        """
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
        """Called by virtual step fields to set pixel move amount for
        virtual robot control
        """
        sender = self.sender()
        if sender == self.step_meter_field:
            self.virtual_robot_meter_step = float(self.step_meter_field.text())
        elif sender == self.step_degree_field:
            self.virtual_robot_degree_step = float(
              self.step_degree_field.text())

    def virtual_move(self): # TODO: make it look nice
        """Called by virtual movement buttons to move robot by
        virtual_robot_step pixels
        """
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
        D.robot.update_sensor_positions()
        D.robot.update_sensor_values(virtual=True, image_map=self.image_map)
        

def sensor_callback( data ):
    """sensor_callback is called for each sensorPacket message to
    receive data from robot"""
    global D
    if not D.use_gui_control:
        D.robot.charge_level = '{:.0%}'.format(data.chargeLevel)
        # data comes in as meters and radians
        D.robot.x['prev'] = D.robot.x['raw']
        D.robot.y['prev'] = D.robot.y['raw']
        D.robot.x['raw'] = data.x
        D.robot.y['raw'] = data.y
        D.robot.t['raw'] = math.degrees(data.theta)
        D.robot.update_sensor_values(values=[data.cliffLeftSignal,
                                             data.cliffFrontLeftSignal,
                                             data.cliffFrontRightSignal,
                                             data.cliffRightSignal])
        Robot.recent_move = (True if D.robot.x['raw'] != D.robot.x['prev'] or
            D.robot.y['raw'] != D.robot.y['prev'] else False)


if __name__ == '__main__':
    import sys
    rospy.init_node('RobotBox')
    # set up a callback for the sensorPacket stream, i.e., "topic"
    rospy.Subscriber('sensorPacket', SensorPacket, sensor_callback)
    app = QtGui.QApplication(sys.argv)
    display = RobotGUI()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
