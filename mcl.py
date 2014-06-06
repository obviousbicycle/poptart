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


# A lot of variables are attached to self when they don't have to be.
# I don't know if I care enough to fix that. Maybe later.
class RobotBox(QtGui.QMainWindow):
    def __init__(self):
        # Construct the parent class
        super(RobotBox, self).__init__()

        # Changing window settings, mainly to remove the maximize button
        self.setWindowFlags(QtCore.Qt.CustomizeWindowHint)
        self.setWindowFlags(QtCore.Qt.WindowCloseButtonHint)

        self.initCentral()
        self.initLeft()
        self.initBottom()
        self.initRight()
        self.initDebug()

        self.showRobotButton.setChecked(True)
        self.ShowRobot()
        self.noTrailButton.setChecked(True)
        self.NoTrail()

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
        self.positionBox = QtGui.QWidget(self.imageBox)

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
        positionLayout = QtGui.QGridLayout(self)
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
        positionTitle.setText("Position data:")
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
        lightBox = QtGui.QWidget(self)

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
        lightLayout = QtGui.QGridLayout(self)
        lightBox.setLayout(lightLayout)

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
        lightTitle.setText("Light sensor readings:")
        lightTitle.setAlignment(QtCore.Qt.AlignHCenter)

        self.lightDock = QtGui.QDockWidget("", self)
        self.lightDock.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea)
        self.lightDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.lightDock.setTitleBarWidget(lightTitle)
        self.lightDock.setWidget(lightBox)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self.lightDock)

    def initRight(self):
        """
        Right Dock Widget: actionBox
        For changing settings and stuff
        """
        # actionBox is just a generic widget holding a grid layout
        # Note: the grid layout is currently unnecessary, but if we expand it
        # later such that it then requires a grid, it'll be less of a headache.
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

        # debug log
        self.debugLogButton = QtGui.QPushButton("Open &debug log", self)
        self.debugLogButton.clicked.connect(self.DebugLog)

        # Display settings
        displayBox = QtGui.QWidget(actionBox)
        displayLayout = QtGui.QGridLayout(displayBox)
        displayBox.setLayout(displayLayout)

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

        displayLayout.addWidget(robotLabel, 0, 0)
        displayLayout.addWidget(self.showRobotButton, 1, 0)
        displayLayout.addWidget(self.hideRobotButton, 1, 1)

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

        displayLayout.addWidget(trailLabel, 2, 0)
        displayLayout.addWidget(self.showTrailButton, 3, 0)
        displayLayout.addWidget(self.hideTrailButton, 3, 1)
        displayLayout.addWidget(self.noTrailButton, 3, 2)
        displayLayout.addWidget(self.eraseTrailButton, 4, 1)

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

        displayLayout.addWidget(MCLLabel, 5, 0)
        displayLayout.addWidget(self.showMCLButton, 6, 0)
        displayLayout.addWidget(self.hideMCLButton, 6, 1)
        displayLayout.addWidget(self.noMCLButton, 6, 2)
        displayLayout.addWidget(self.eraseMCLButton, 7, 1)

        # Putting together layout and dock widget
        actionLayout.addWidget(self.saveButton, 0, 0)
        actionLayout.addWidget(self.loadButton, 1, 0)
        actionLayout.addWidget(self.clearButton, 2, 0)
        actionLayout.addWidget(self.debugLogButton, 3, 0)
        actionLayout.addWidget(displayBox, 4, 0)

        self.actionDock = QtGui.QDockWidget("", self)
        self.actionDock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        self.actionDock.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.actionDock.setTitleBarWidget(QtGui.QWidget(self))
        self.actionDock.setWidget(actionBox)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.actionDock)

    def initDebug(self):
        """ creates debug log """
        self.debugWindow = QtGui.QWidget(None)
        debugLayout = QtGui.QGridLayout(self.debugWindow)
        self.debugWindow.setLayout(debugLayout)
        self.debugWindow.setWindowTitle("Debug")
        self.debugWindow.setWindowFlags(QtCore.Qt.CustomizeWindowHint)
        self.debugWindow.setWindowFlags(QtCore.Qt.WindowSystemMenuHint)

        self.debugLog = QtGui.QTextEdit(self.debugWindow)
        self.debugLog.setReadOnly(True)
        
        self.debugPrint = False
        self.printButton = QtGui.QPushButton("Print", self.debugWindow)
        self.printButton.clicked.connect(self.DebugPrint)

        debugLayout.addWidget(self.printButton, 0, 0)
        debugLayout.addWidget(self.debugLog, 1, 0)

    def PrintToLog(self, string):
        """ prints stuff in debug log """
        if self.debugPrint:
            self.debugLog.append(string)

    def RedrawCallback(self):
        global D

        self.imageLock.acquire()
        try:
            self.PrintToLog("--- GUI UPDATE ---")

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

            self.PrintToLog("D.x: " + str(D.x) + " | D.xDiff: " + str(D.xDiff) + " | xDisplay: " + str(xDisplay))
            self.PrintToLog("D.y: " + str(D.y) + " | D.yDiff: " + str(D.yDiff) + " | yDisplay: " + str(yDisplay))
            self.PrintToLog("D.theta: " + str(D.x) + " | D.thetaDiff: " + str(D.thetaDiff) + " | thetaDisplay: " + str(thetaDisplay))

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
                self.PrintToLog("-- MCL UPDATE --")
                pDensity = int(area/100)
                if D.particles == []:
                    self.PrintToLog("- INITIAL SETUP -")
                    for n in range(pDensity-1):
                        # a point with random x, y, theta is generated
                        # initially, every point has an equal probability of
                        # being the robot's actual location
                        p = [random.randint(0, WIDTH-1),\
                          random.randint(0, HEIGHT-1),\
                          random.randint(0, 360),\
                          1.0/pDensity]
                        D.particles.append(p)
                    D.particles.append([location[0],location[1],location[2],1.0/pDensity])
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
                        self.PrintToLog("Pre-move probs: " + str(map(lambda p: p[3], oldGen)))
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
                        self.PrintToLog("Post-move probs: " + str(map(lambda p: p[3], oldGen)))
                        oldProbSum = reduce(lambda m,n: m+n,\
                          map(lambda p: p[3], oldGen))
                        # begin populating new generation with copies from
                        # old generation, based on the points' probabilities
                        oldPtSet = []
                        for n in range(pDensity):
                            # dart-throwing approach
                            dart = random.uniform(0, oldProbSum)
                            counter = 0
                            cumulativeProb = oldGen[1][3]
                            while dart < cumulativeProb:
                                cumulativeProb += oldGen[counter][3]
                                counter += 1
                            newGen.append(oldGen[counter])
                            if oldGen[counter] not in oldPtSet:
                                oldPtSet.append(oldGen[counter])
                        # recalculate probabilities
                        adjusted = []
                        for n in range(len(oldPtSet)):
                            toCheck = oldPtSet.pop()
                            instances = filter(lambda p: p == toCheck, newGen)
                            adjusted+=map(lambda p: [p[0],p[1],p[2],len(instances)/pDensity], instances)
                        newGen = adjusted
                        # add some noise to each new point
                        noise = 4.0
                        noiseTheta = 2.5
                        newGen = map(lambda p: [random.gauss(p[0],noise),\
                                                random.gauss(p[1],noise),\
                                                random.gauss(p[2],noiseTheta),\
                                                p[3]],
                                                newGen)
                        # newGen is now ready
                        self.PrintToLog("New gen probs: " + str(map(lambda p: p[3], newGen)))
                        D.particles = newGen
                    else:
                        self.PrintToLog("- NO CHANGE -")
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
    def DebugLog(self):
        """ receives click signal from debugLogButton """
        if self.debugWindow.isVisible():
            self.debugPrint = False
            self.debugLogButton.setText("Open debug log")
            self.debugWindow.close()
        else:
            self.debugLogButton.setText("Close debug log")
            self.debugWindow.show()

    @Slot()
    def DebugPrint(self):
        """ receives click signal from printButton """
        if self.debugWindow.isVisible() and self.debugPrint:
            self.debugPrint = False
            self.printButton.setText("Print")
        else:
            self.debugPrint = True
            self.printButton.setText("Stop")

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
        self.eraseTrailButton.setEnabled(False)
        self.showMCLButton.setEnabled(False)
        self.hideMCLButton.setEnabled(False)
        self.noMCLButton.setChecked(True)
        self.NoMCL()
        D.location = []
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
        self.eraseMCLButton.setEnabled(False)
        D.particles = []
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
