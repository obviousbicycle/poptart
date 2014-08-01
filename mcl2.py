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
        left_tabs = QtGui.QTabWidget(self)

        # Tab for displaying position info and virtual control buttons
        position_group = QtGui.QWidget(self)
        # Coordinate display
        x_label = QtGui.QLabel("X (" + USE_CM*"c" + "m): ", self)
        self.x_value = QtGui.QLabel(self)
        y_label = QtGui.QLabel("Y (" + USE_CM*"c" + "m): ", self)
        self.y_value = QtGui.QLabel(self)
        t_label = QtGui.QLabel("Theta (deg): ", self)
        self.t_value = QtGui.QLabel(self)
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
        virtual_layout = QtGui.QGridLayout()
        self.virtual_group.setLayout(virtual_layout)
        virtual_widgets_to_add = [
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
        for args in virtual_widgets_to_add: virtual_layout.addWidget(*args)
        virtual_layout.setColumnMinimumWidth(3, WIDGET_SPACING)
        virtual_layout.setColumnMinimumWidth(6, WIDGET_SPACING)
        virtual_layout.setColumnStretch(0, 1)
        virtual_layout.setColumnStretch(20, 1)
        # Assembling position info sublayout
        physical_group = QtGui.QWidget(self)
        physical_layout = QtGui.QGridLayout()
        physical_group.setLayout(physical_layout)
        physical_widgets_to_add = [
            (x_label, 1, 1, QtCore.Qt.AlignRight),
            (self.x_value, 1, 2),
            (self.x_reset_button, 2, 1, 1, 2, QtCore.Qt.AlignHCenter),
            (y_label, 4, 1, QtCore.Qt.AlignRight),
            (self.y_value, 4, 2),
            (self.y_reset_button, 5, 1, 1, 2, QtCore.Qt.AlignHCenter),
            (t_label, 1, 4, QtCore.Qt.AlignRight),
            (self.t_value, 1, 5),
            (self.theta_reset_button, 2, 4, 1, 2, QtCore.Qt.AlignHCenter),
            (self.all_reset_button, 4, 4, 2, 2, QtCore.Qt.AlignHCenter),
            (self.virtual_control_checkbox, 7, 1, 1, 5),
            (self.virtual_group, 8, 1, 1, 5)
        ]
        for args in physical_widgets_to_add: physical_layout.addWidget(*args)
        physical_layout.setColumnMinimumWidth(3, WIDGET_SPACING)
        physical_layout.setRowMinimumHeight(6, WIDGET_SPACING)
        physical_layout.setColumnStretch(0, 1)
        physical_layout.setColumnStretch(6, 1)
        physical_layout.addWidget
        # Assembling layout for position_group
        position_group.setLayout(physical_layout)
        # Virtual control initially set to off
        self.virtual_control_toggle()

        # IR light sensor info tab
        light_group = QtGui.QWidget(self)
        position_label = QtGui.QLabel("Position", self)
        readings_label = QtGui.QLabel("Readings", self)
        lower_thresholds_label = QtGui.QLabel("Lower\nthresholds", self)
        lower_thresholds_label.setAlignment(QtCore.Qt.AlignHCenter)
        upper_thresholds_label = QtGui.QLabel("Upper\nthresholds", self)
        upper_thresholds_label.setAlignment(QtCore.Qt.AlignHCenter)
        back_left_label = QtGui.QLabel("Back\nleft", self)
        back_left_label.setAlignment(QtCore.Qt.AlignHCenter)
        front_left_label = QtGui.QLabel("Front\nleft", self)
        front_left_label.setAlignment(QtCore.Qt.AlignHCenter)
        front_right_label = QtGui.QLabel("Front\nright", self)
        front_right_label.setAlignment(QtCore.Qt.AlignHCenter)
        back_right_label = QtGui.QLabel("Back\nright", self)
        back_right_label.setAlignment(QtCore.Qt.AlignHCenter)
        # Display values
        self.back_left_position = QtGui.QLabel(self)
        self.front_left_position = QtGui.QLabel(self)
        self.front_right_position = QtGui.QLabel(self)
        self.back_right_position = QtGui.QLabel(self)
        self.back_left_value = QtGui.QLabel(self)
        self.front_left_value = QtGui.QLabel(self)
        self.front_right_value = QtGui.QLabel(self)
        self.back_right_value = QtGui.QLabel(self)
        # Light threshold entry
        validator = QtGui.QIntValidator(0, 9999, self)
        self.back_left_lower_field = QtGui.QLineEdit(
          str(self.thresholds[0]))
        self.front_left_lower_field = QtGui.QLineEdit(
          str(self.thresholds[1]))
        self.front_right_lower_field = QtGui.QLineEdit(
          str(self.thresholds[2]))
        self.back_right_lower_field = QtGui.QLineEdit(
          str(self.thresholds[3]))
        self.back_left_upper_field = QtGui.QLineEdit(
          str(self.thresholds[4]))
        self.front_left_upper_field = QtGui.QLineEdit(
          str(self.thresholds[5]))
        self.front_right_upper_field = QtGui.QLineEdit(
          str(self.thresholds[6]))
        self.back_right_upper_field = QtGui.QLineEdit(
          str(self.thresholds[7]))
        light_fields = [
            self.back_left_lower_field, self.front_left_lower_field,
            self.front_right_lower_field, self.back_right_lower_field,
            self.back_left_upper_field, self.front_left_upper_field,
            self.front_right_upper_field, self.back_right_upper_field
        ]
        for f in light_fields:
            f.editingFinished.connect(self.light_threshold_set)
            f.setValidator(validator)
        # Assembling layout for light_group
        light_layout = QtGui.QGridLayout()
        light_group.setLayout(light_layout)
        light_widgets_to_add = [
                        (position_label, 1, 3, QtCore.Qt.AlignHCenter),
                        (readings_label, 1, 5, QtCore.Qt.AlignHCenter),
                 (lower_thresholds_label, 1, 7, QtCore.Qt.AlignHCenter),
                 (upper_thresholds_label, 1, 9, QtCore.Qt.AlignHCenter),
                        (back_left_label, 3, 1, QtCore.Qt.AlignRight),
                (self.back_left_position, 3, 3, QtCore.Qt.AlignHCenter),
                   (self.back_left_value, 3, 5, QtCore.Qt.AlignHCenter),
              (self.back_left_lower_field, 3, 7, QtCore.Qt.AlignHCenter),
              (self.back_left_upper_field, 3, 9, QtCore.Qt.AlignHCenter),
                       (front_left_label, 5, 1, QtCore.Qt.AlignRight),
               (self.front_left_position, 5, 3, QtCore.Qt.AlignHCenter),
                  (self.front_left_value, 5, 5, QtCore.Qt.AlignHCenter),
             (self.front_left_lower_field, 5, 7, QtCore.Qt.AlignHCenter),
             (self.front_left_upper_field, 5, 9, QtCore.Qt.AlignHCenter),
                      (front_right_label, 7, 1, QtCore.Qt.AlignRight),
              (self.front_right_position, 7, 3, QtCore.Qt.AlignHCenter),
                 (self.front_right_value, 7, 5, QtCore.Qt.AlignHCenter),
            (self.front_right_lower_field, 7, 7, QtCore.Qt.AlignHCenter),
            (self.front_right_upper_field, 7, 9, QtCore.Qt.AlignHCenter),
                       (back_right_label, 9, 1, QtCore.Qt.AlignRight),
               (self.back_right_position, 9, 3, QtCore.Qt.AlignHCenter),
                  (self.back_right_value, 9, 5, QtCore.Qt.AlignHCenter),
             (self.back_right_lower_field, 9, 7, QtCore.Qt.AlignHCenter),
             (self.back_right_upper_field, 9, 9, QtCore.Qt.AlignHCenter)
        ]
        for args in light_widgets_to_add: light_layout.addWidget(*args)
        light_layout.setRowStretch(10, 1)
        light_layout.setColumnMinimumWidth(2, WIDGET_SPACING)
        light_layout.setColumnMinimumWidth(4, WIDGET_SPACING)
        light_layout.setColumnMinimumWidth(6, WIDGET_SPACING) 
        light_layout.setColumnMinimumWidth(8, WIDGET_SPACING)

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
        left_tabs.addTab(position_group, "Position")
        left_tabs.addTab(light_group, "IR sensors")
        left_tabs.addTab(particle_group, "Particles")
        left_tabs.setMinimumWidth(450)
        self.left_dock = QtGui.QDockWidget("", self)
        self.left_dock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
        self.left_dock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.left_dock.setTitleBarWidget(QtGui.QWidget(self))
        self.left_dock.setWidget(left_tabs)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.left_dock)

    def init_right(self):
        """Right Dock Widget: display_group, parameterGroup, actionGroup
        display_group changes settings and stuff
        parameterGroup changes parameters and stuff
        actionGroup has image actions
        """
        right_tabs = QtGui.QTabWidget(self)

        # Image display settings tab
        display_group = QtGui.QWidget(self)
        self.robot_marker_checkbox = QtGui.QCheckBox("Robot marker", self)
        self.robot_marker_checkbox.setChecked(True)
        self.axes_checkbox = QtGui.QCheckBox("Axes", self)
        self.grid_checkbox = QtGui.QCheckBox("Grid", self)
        # Trail draw settings
        trail_label = QtGui.QLabel("Robot trail", self)
        self.show_trail_button = QtGui.QRadioButton("Show", self)
        self.show_trail_button.clicked.connect(self.set_trail)
        self.hide_trail_button = QtGui.QRadioButton("Hide", self)
        self.hide_trail_button.clicked.connect(self.set_trail)
        self.no_trail_button = QtGui.QRadioButton("Off", self)
        self.no_trail_button.clicked.connect(self.set_trail)
        self.erase_trail_button = QtGui.QPushButton("Erase", self)
        self.erase_trail_button.clicked.connect(self.erase_trail)
        # Limit radio button choices to trail-related buttons
        trail_group = QtGui.QButtonGroup(self)
        trail_group.addButton(self.show_trail_button)
        trail_group.addButton(self.hide_trail_button)
        trail_group.addButton(self.no_trail_button)
        # MCL draw settings
        mcl_label = QtGui.QLabel("Monte Carlo", self)
        self.show_mcl_button = QtGui.QRadioButton("Show", self)
        self.show_mcl_button.clicked.connect(self.set_mcl)
        self.hide_mcl_button = QtGui.QRadioButton("Hide", self)
        self.hide_mcl_button.clicked.connect(self.set_mcl)
        self.no_mcl_button = QtGui.QRadioButton("Off", self)
        self.no_mcl_button.clicked.connect(self.set_mcl)
        self.erase_mcl_button = QtGui.QPushButton("Erase", self)
        self.erase_mcl_button.clicked.connect(self.erase_mcl)
        # Limit radio button choices to MCL-related buttons
        mcl_group = QtGui.QButtonGroup(self)
        mcl_group.addButton(self.show_mcl_button)
        mcl_group.addButton(self.hide_mcl_button)
        mcl_group.addButton(self.no_mcl_button)
        # Assembling layout for display_group
        display_layout = QtGui.QGridLayout()
        display_group.setLayout(display_layout)
        display_widgets_to_add = [
            (self.robot_marker_checkbox, 1, 1, 1, 2),
                    (self.axes_checkbox, 2, 1),
                    (self.grid_checkbox, 2, 2),
                           (trail_label, 4, 1, 1, 2),
                (self.show_trail_button, 5, 1),
                (self.hide_trail_button, 5, 2),
                  (self.no_trail_button, 5, 3),
               (self.erase_trail_button, 6, 1, 1, 3, QtCore.Qt.AlignLeft),
                             (mcl_label, 8, 1, 1, 2),
                  (self.show_mcl_button, 9, 1),
                  (self.hide_mcl_button, 9, 2),
                    (self.no_mcl_button, 9, 3),
                 (self.erase_mcl_button, 10, 1, 1, 3, QtCore.Qt.AlignLeft)
        ]
        for args in display_widgets_to_add: display_layout.addWidget(*args)
        display_layout.setRowMinimumHeight(2, WIDGET_SPACING)
        display_layout.setRowMinimumHeight(6, WIDGET_SPACING)
        display_layout.setRowStretch(20, 1)
        # Set initial button states
        self.no_trail_button.setChecked(True)
        self.no_mcl_button.setChecked(True)
        self.erase_trail_button.setEnabled(False)
        self.erase_mcl_button.setEnabled(False)

        # Robot marker offset tab
        offset_group = QtGui.QWidget(self)
        # X offset text field
        x_offset_label = QtGui.QLabel("X", self)
        self.x_offset_field = QtGui.QLineEdit("0")
        self.x_offset_field.editingFinished.connect(self.offset_field_change)
        x_offset_validator = QtGui.QIntValidator(
          -self.width/2, self.width/2, self)
        self.x_offset_field.setValidator(x_offset_validator)
        # X offset slider
        self.x_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.x_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.x_offset_slider.setMinimum(-self.width)
        self.x_offset_slider.setMaximum(self.width)
        self.x_offset_slider.setTickInterval(self.width/10)
        self.x_offset_slider.setValue(0)
        self.x_offset_slider.valueChanged.connect(self.offset_slider_change)
        # Y offset text field
        y_offset_label = QtGui.QLabel("Y", self)
        self.y_offset_field = QtGui.QLineEdit("0")
        self.y_offset_field.editingFinished.connect(self.offset_field_change)
        y_offset_validator = QtGui.QIntValidator(
          -self.height / 2, self.height / 2, self)
        self.y_offset_field.setValidator(y_offset_validator)
        # Y offset slider
        self.y_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.y_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.y_offset_slider.setMinimum(-self.height)
        self.y_offset_slider.setMaximum(self.height)
        self.y_offset_slider.setTickInterval(self.height/10)
        self.y_offset_slider.setValue(0)
        self.y_offset_slider.valueChanged.connect(self.offset_slider_change)
        # Theta offset text field
        t_offset_label = QtGui.QLabel("Theta", self)
        self.t_offset_field = QtGui.QLineEdit("0")
        self.t_offset_field.editingFinished.connect(
          self.offset_field_change)
        t_offset_validator = QtGui.QIntValidator(0, 359, self)
        self.t_offset_field.setValidator(t_offset_validator)
        # Theta offset slider
        self.t_offset_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.t_offset_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.t_offset_slider.setMinimum(0)
        self.t_offset_slider.setMaximum(359)
        self.t_offset_slider.setTickInterval(15)
        self.t_offset_slider.setValue(0)
        self.t_offset_slider.valueChanged.connect(
          self.offset_slider_change)
        # offset_group layout
        offset_layout = QtGui.QGridLayout()
        offset_group.setLayout(offset_layout)
        offset_widgets_to_add = [
                      (x_offset_label, 3, 0),
                 (self.x_offset_field, 3, 1),
                (self.x_offset_slider, 4, 0, 1, 2),
                      (y_offset_label, 5, 0),
                 (self.y_offset_field, 5, 1),
                (self.y_offset_slider, 6, 0, 1, 2),
                      (t_offset_label, 7, 0),
                 (self.t_offset_field, 7, 1),
                (self.t_offset_slider, 8, 0, 1, 2)
        ]
        for args in offset_widgets_to_add: offset_layout.addWidget(*args)
        offset_layout.setColumnStretch(0, 1)
        offset_layout.setRowStretch(20, 1)
        
        # MCL tab
        mcl_group = QtGui.QWidget(self)
        num_particles_label = QtGui.QLabel("# particles", self)
        # Number of MCL particles nput field
        self.num_particles_field = QtGui.QLineEdit(str(self.num_particles))
        self.num_particles_field.editingFinished.connect(self.num_particles_set)
        num_particles_max = self.width*self.height
        num_particles_validator = QtGui.QIntValidator(
          1, num_particles_max, self)
        self.num_particles_field.setValidator(num_particles_validator)
        self.num_particles_field.setMaxLength(len(str(num_particles_max)))
        # XY noise text field
        # Apparently sliders only accept integer values??
        xy_noise_label = QtGui.QLabel("XY noise", self)
        self.xy_noise_field = QtGui.QLineEdit(str(self.xy_noise))
        self.xy_noise_field.editingFinished.connect(self.noise_field_change)
        xy_noise_max = 10.0
        xy_noise_validator = QtGui.QDoubleValidator(
          0.0, xy_noise_max, 1, self)
        self.xy_noise_field.setValidator(xy_noise_validator)
        self.xy_noise_field.setMaxLength(len(str(xy_noise_max)))
        # XY noise slider
        self.xy_noise_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.xy_noise_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.xy_noise_slider.setMinimum(0)
        self.xy_noise_slider.setMaximum(10*xy_noise_max)
        self.xy_noise_slider.setTickInterval(5)
        self.xy_noise_slider.setValue(10*self.xy_noise)
        self.xy_noise_slider.valueChanged.connect(self.noise_slider_change)
        # Theta noise text field
        t_noise_label = QtGui.QLabel("Theta noise", self)
        self.t_noise_field = QtGui.QLineEdit(str(self.t_noise))
        self.t_noise_field.editingFinished.connect(self.noise_field_change)
        t_noise_max = 5.0
        t_noise_validator = QtGui.QDoubleValidator(
          0.0, t_noise_max, 1, self)
        self.t_noise_field.setValidator(t_noise_validator)
        self.t_noise_field.setMaxLength(len(str(t_noise_max)))
        # Theta noise slider
        self.t_noise_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.t_noise_slider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.t_noise_slider.setMinimum(0)
        self.t_noise_slider.setMaximum(10*t_noise_max)
        self.t_noise_slider.setTickInterval(5)
        self.t_noise_slider.setValue(10*self.t_noise)
        self.t_noise_slider.valueChanged.connect(self.noise_slider_change)
        # Checkboxes for particle draw settings
        self.particle_detail_checkbox = QtGui.QCheckBox(
          "Detailed particles", self)
        self.particle_coloring_checkbox = QtGui.QCheckBox(
          "Variable particle colors", self)
        # mcl_group layout
        mcl_layout = QtGui.QGridLayout()
        mcl_group.setLayout(mcl_layout)
        mcl_widgets_to_add = [
                        (num_particles_label, 3, 0),
                   (self.num_particles_field, 3, 1),
                             (xy_noise_label, 5, 0),
                        (self.xy_noise_field, 5, 1),
                       (self.xy_noise_slider, 6, 0, 1, 2),
                              (t_noise_label, 7, 0),
                         (self.t_noise_field, 7, 1),
                        (self.t_noise_slider, 8, 0, 1, 2),
              (self.particle_detail_checkbox, 9, 0, 1, 2),
            (self.particle_coloring_checkbox, 10, 0, 1, 2)
        ]
        for args in mcl_widgets_to_add: mcl_layout.addWidget(*args)
        mcl_layout.setColumnStretch(0, 1)
        mcl_layout.setRowStretch(20, 1)
        # Initial checkboxes settings
        self.particle_detail_checkbox.setChecked(True)
        self.particle_coloring_checkbox.setChecked(True)

        # Putting everything together
        right_tabs.addTab(display_group, "Draw")
        right_tabs.addTab(offset_group, "Offset")
        right_tabs.addTab(mcl_group, "MCL")
        self.right_dock = QtGui.QDockWidget("", self)
        self.right_dock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        self.right_dock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.right_dock.setTitleBarWidget(QtGui.QWidget(self))
        self.right_dock.setWidget(right_tabs)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.right_dock)

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
            x_offset = self.left_dock.width() + 6
            y_offset = (max(self.left_dock.height(), self.right_dock.height()) -
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
        self.t_value.setText('{:.1f}'.format(D.robot.t['display']))
        self.x_value.setText('{:.2f}'.format(D.robot.x['display']))
        self.y_value.setText('{:.2f}'.format(D.robot.y['display']))
        D.robot.update_draw(self.width, self.height)
        location = (D.robot.x['draw'], D.robot.y['draw'], D.robot.t['display'])
        if not self.trail or location != self.trail[-1]:
            self.trail.append(location)
            if not self.make_trail and len(self.trail) > 2:
                del self.trail[0]

        # Set sensor info display
        sensor_global = D.robot.get_sensor_global_position()
        self.back_left_position.setText('({:.2f}, {:.2f})'.format(
          sensor_global[0].x, sensor_global[0].y))
        self.front_left_position.setText('({:.2f}, {:.2f})'.format(
          sensor_global[1].x, sensor_global[1].y))
        self.front_right_position.setText('({:.2f}, {:.2f})'.format(
          sensor_global[2].x, sensor_global[2].y))
        self.back_right_position.setText('({:.2f}, {:.2f})'.format(
          sensor_global[3].x, sensor_global[3].y))
        self.back_left_value.setText(str(D.robot.sensors[0].light))
        self.front_left_value.setText(str(D.robot.sensors[1].light))
        self.front_right_value.setText(str(D.robot.sensors[2].light))
        self.back_right_value.setText(str(D.robot.sensors[3].light))

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
            old_sum_prob = math.fsum([p.prob for p in self.particles])
            if old_sum_prob <= 0.01:
                # If all the points are really unlikely, just start over
                self.particles = []
            else: # TODO
                # Resampling creates an entirely new list of particles
                # based on the probabilities from the old generation
                new_gen = []
                probs = [p.prob for p in self.particles]
                cumulative_prob = [math.fsum(probs[i::-1]) for i in xrange(
                  self.num_particles)]
                counter = 0
                for i in xrange(self.num_particles):
                    # Step through probability "blocks" of particle list
                    new_gen.append(self.particles[counter])
                    while (i * old_sum_prob / self.num_particles >
                            cumulative_prob[counter]):
                        counter += 1
                new_sum_prob = math.fsum([p.prob for p in new_gen])
                self.particles = new_gen
                for p in self.particles:
                    # Resampled particles' probabilities normalized
                    p.finish_resample(
                      self.xy_noise, self.t_noise, new_sum_prob)
        
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
        if self.axes_checkbox.isChecked():
            painter.setPen(QtGui.QColor.fromHsv(240, 63, 63))
            painter.setBrush(QtGui.QColor.fromHsv(240, 63, 63))
            painter.drawLine(self.width/2, 0, self.width/2, self.height)
            painter.drawLine(0, self.height/2, self.width, self.height/2)
        # Drawing grid
        if self.grid_checkbox.isChecked():
            spacing = 20
            painter.setPen(QtGui.QColor.fromHsv(240,63,63))
            painter.setBrush(QtGui.QColor.fromHsv(240,63,63))
            # grid is centered on origin
            for x in xrange((self.width/2)%spacing, self.width, spacing):
                for y in xrange((self.height/2)%spacing, self.height, spacing):
                    painter.drawEllipse(x, y, 1, 1)
        # Drawing particles
        if self.make_mcl == 2:
            self.particle_radius = (3 if
              self.particle_detail_checkbox.isChecked() else 2)
            selected_radius = self.particle_radius + 2
            get_color = (self.calculate_color if
                self.particle_coloring_checkbox.isChecked() else
                lambda x, y, z: QtGui.QColor("darkGray"))
            for p in self.particles:
                color = get_color(p, 255, 180)
                painter.setPen(color)
                painter.setBrush(color)
                painter.drawEllipse(QtCore.QPoint(p.x['draw'], p.y['draw']), 
                  self.particle_radius, self.particle_radius)
            if self.particle_detail_checkbox.isChecked():
                # Draw particle sensors and particle heading
                for p in self.particles:
                    painter.setPen("black")
                    sensor_draw = p.get_sensor_draw_position()
                    for s in sensor_draw:
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
                sensor_draw = Particle.selected.get_sensor_draw_position()
                for s in sensor_draw:
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
        if self.robot_marker_checkbox.isChecked():
            marker_radius = 8
            painter.setPen(QtGui.QColor("black"))
            painter.setBrush(QtGui.QColor("lightGray"))
            painter.drawEllipse(
              QtCore.QPoint(self.trail[-1][0], self.trail[-1][1]),
              marker_radius, marker_radius)
            # Sensors
            painter.setPen(QtGui.QColor(0,255,0))
            robot_sensor_draw = D.robot.get_sensor_draw_position()
            for s in robot_sensor_draw:
                painter.drawPoint(s.x, s.y)
            # Pointer
            painter.drawLine(self.calculate_pointer(
              self.trail[-1][0], self.trail[-1][1],
              float(self.t_value.text()), marker_radius + 3, 8))
        painter.end()
        return image

    def calculate_pointer(self, x, y, theta, distance, length):
        """Calculates a line with the given length and an angle of
        theta degrees, offset from (x, y) by distance
        """
        x_distance = distance * math.sin(math.radians(theta + 90.0))
        y_distance = distance * math.cos(math.radians(theta + 90.0))
        x_length = x_distance + length*math.sin(math.radians(theta + 90.0))
        y_length = y_distance + length*math.cos(math.radians(theta + 90.0))
        return QtCore.QLineF(x_distance + x, y_distance + y,
                             x_length + x, y_length + y)

    def calculate_color(self, p, saturation, value):
        """Determines the color of a particle"""
        # The color of one point is based on how many other points
        # are close to it, "close" here being defined very arbitrarily
        near = 10 # pixels
        num_close_points = len(filter(
          lambda q: near >= abs(q.x['display'] - p.x['display']) and
                    near >= abs(q.y['display'] - p.y['display']),
          self.particles))
        hue = 300 * (1 - num_close_points/(self.num_particles*0.75))
        if hue < 0.0: hue = 0
        if hue > 300.0: hue = 300
        return QtGui.QColor.fromHsv(hue, saturation, value)

    ###
    # MCL parameter slots
    ###
    def noise_slider_change(self):
        """Called by self.xy_noise_slider and self.t_noise_slider to
        change noise parameters and text fields
        """
        sender = self.sender()
        if sender == self.xy_noise_slider:
            self.xy_noise = self.xy_noise_slider.value() / 10.0
            self.xy_noise_field.setText(str(self.xy_noise))
        elif sender == self.t_noise_slider:
            self.t_noise = self.t_noise_slider.value() / 10.0
            self.t_noise_field.setText(str(self.t_noise))

    def noise_field_change(self):
        """Called by self.xy_noise_field and self.t_noise_field to
        change noise parameters and sliders
        """
        sender = self.sender()
        if sender == self.xy_noise_field:
            self.xy_noise = float(self.xy_noise_field.text())
            self.xy_noise_slider.blockSignals(True)
            self.xy_noise_slider.setSliderPosition(10 * self.xy_noise)
            self.xy_noise_slider.blockSignals(False)
        elif sender == self.t_noise_field:
            self.t_noise = float(self.t_noise_field.text())
            self.t_noise_slider.blockSignals(True)
            self.t_noise_slider.setSliderPosition(10 * self.t_noise)
            self.t_noise_slider.blockSignals(False)

    def num_particles_set(self):
        """Called by num_particles_field to change number of MCL
        particles
        """
        self.num_particles = int(self.num_particles_field.text())

    ###
    # Light sensor slot
    ###
    def light_threshold_set(self):
        """Called by light threshold field widgets to change light
        thresholds
        """
        sender = self.sender()
        corresponding = {
            self.back_left_lower_field: 0,
            self.front_left_lower_field: 1,
            self.front_right_lower_field: 2,
            self.back_right_lower_field: 3,
            self.back_left_upper_field: 4,
            self.front_left_upper_field: 5,
            self.front_right_upper_field: 6,
            self.back_right_upper_field: 7
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
        to_load = QtGui.QImage()
        dialog = QtGui.QFileDialog(self)
        dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
        dialog.setViewMode(QtGui.QFileDialog.Detail)
        fname, _ = dialog.getOpenFileName(
          self, "Open image (features must be black and/or white)",
          "/home/robotics/Desktop/", "*.png")
        if to_load.load(fname):
            self.width = to_load.width()
            self.height = to_load.height()
            self.image_map = to_load
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
        if sender == self.show_trail_button:
            self.erase_trail_button.setEnabled(True)
            self.make_trail = 2
        elif sender == self.hide_trail_button:
            self.erase_trail_button.setEnabled(True)
            self.make_trail = 1
        elif sender == self.no_trail_button:
            self.erase_trail()
            self.erase_trail_button.setEnabled(False)
            self.make_trail = 0
    
    def erase_trail(self): self.trail = []

    def set_mcl(self):
        """Called by buttons that change MCL display settings"""
        sender = self.sender()
        if sender == self.show_mcl_button:
            self.erase_mcl_button.setEnabled(True)
            self.make_mcl = 2
        elif sender == self.hide_mcl_button:
            self.erase_mcl_button.setEnabled(True)
            self.make_mcl = 1
        elif sender == self.no_mcl_button:
            self.erase_mcl()
            self.erase_mcl_button.setEnabled(False)
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
