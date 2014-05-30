#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
from ControllerParser import update_D
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import time
import math


####
# lab1.py ~ starter file for scripting the Create with ROS
####


####
# D is our global system state
####

class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state

D.baseVel = 100
D.wheelToStick = True

# the game controller's buttons and pads ~ initial values...
D.BTNS = [0]*16
D.PADS = [0]*16


####
# main and init
####


def main():
    """ the main program that gives our node a name,
       sets up service objects, subscribes to topics (with a callback),
       and then lets interactions happen!
    """
    global D

    # set up services and subscribe to data streams
    init()

    rospy.spin()
    

def init():
    """ returns an object (tank) that allows you
       to set the velocities of the robot's wheels
    """
    global D # to hold system state

    # we need to give our program a ROS node name
    # the name is not important, so we use "lab1_node"
    rospy.init_node('lab1_node', anonymous=True)
    
    # we obtain the tank service
    rospy.wait_for_service('tank') # wait until the motors are available
    D.tank = rospy.ServiceProxy('tank', Tank) # D.tank is our "driver"
    
    # we obtain the song service
    rospy.wait_for_service('song') # wait until our voice is available
    D.song = rospy.ServiceProxy('song', Song) # D.song is our "speaker" 

    rospy.Subscriber( 'controller_data', String, controller_callback )




####
# Controller input
####

def controller_callback(data):
    global D
    message = data.data
    update_D(D, message)
    # print the PADS and BTNS
    #print "PADS is", D.PADS
    #print "BTNS is", D.BTNS
    
    # Joystick control
    # D.PADS[0] and D.PADS[1] refer to x- and y-values of left joystick
    # D.PADS[2] and D.PADS[3] refer to x- and y-values of right joystick
    if D.wheelToStick and D.PADS[0:4] != [0,0,0,0]:
        factor = 0.5
        # Note that one of the velocities will exceed speedCap when turning
        leftVel = int( D.baseVel*(-D.PADS[1] + factor*D.PADS[0]) )
        rightVel = int( D.baseVel*(-D.PADS[3] - factor*D.PADS[2]) )

        print leftVel, rightVel
        D.tank(leftVel, rightVel)
    elif not D.wheelToStick and D.PADS[0:2] != [0,0]:
        leftVel = int( D.baseVel * (-D.PADS[1] + D.PADS[0]) )
        rightVel = int( D.baseVel * (-D.PADS[1] - D.PADS[0]) )

        print leftVel, rightVel
        D.tank(leftVel, rightVel)

    # Thumbpad control
    elif D.PADS[5] == -1.0: # up
        D.tank(D.baseVel, D.baseVel)
    elif D.PADS[5] == 1.0: # down
        D.tank(-D.baseVel, -D.baseVel)
    elif D.PADS[4] == -1.0: # left
        D.tank(-D.baseVel, D.baseVel)
    elif D.PADS[4] == 1.0: # right
        D.tank(D.baseVel, -D.baseVel)

    else:
        D.tank(0,0)

    # Shoulder buttons D.BTNS[4:8]
    speedCap = 200
    smallIncrement = 10
    bigIncrement = 20
    if D.BTNS[4] == 1.0:
        if D.baseVel >= -speedCap + smallIncrement:
            D.baseVel -= smallIncrement
            print 'Base speed decreased to', D.baseVel
        else:
            print 'Base speed at minimum -' + str(speedCap) + \
                    '. Cannot decrease further.'
    elif D.BTNS[5] == 1.0:
        if D.baseVel <= speedCap - smallIncrement:
            D.baseVel += smallIncrement
            print 'Base speed increased to', D.baseVel
        else:
            print 'Base speed at maximum ' + str(speedCap) + \
                    '. Cannot increase further.'
    # Bigger changes for back shoulder buttons
    elif D.BTNS[6] == 1.0:
        if D.baseVel >= -speedCap + bigIncrement:
            D.baseVel -= bigIncrement
            print 'Base speed decreased to', D.baseVel
        else:
            print 'Base speed at minimum -' + str(speedCap) + \
                    '. Cannot decrease further.'
    elif D.BTNS[7] == 1.0:
        if D.baseVel <= speedCap - bigIncrement:
            D.baseVel += bigIncrement
            print 'Base speed increased to', D.baseVel
        else:
            print 'Base speed at maximum ' + str(speedCap) + \
                    '. Cannot increase further.'

    # Front buttons D.BTNS[0:4]
    if D.BTNS[3] == 1.0: # button 4 sings
        D.song([76,76,30,76,30,72,76,30,79], [15,15,15,15,15,15,15,15,15])
    if D.BTNS[0] == 1.0: # button 1 toggles analog stick modes
        D.wheelToStick = not D.wheelToStick
        print 'Changed joystick controls.'
        if D.wheelToStick:
            print 'Left stick controls left wheel, right stick controls right wheel.'
        else:
            print "Left stick up/down axis controls wheels' shared velocity."
            print "Left stick left/right axis controls difference in velocities."



####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
#### 

if __name__ == "__main__":
   main()