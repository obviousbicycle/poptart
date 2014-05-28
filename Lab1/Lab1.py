#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
from ControllerParser import update_D ''' ADDED '''
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

D.leftVel = 50
D.rightVel = 50
D.delta = 5

''' ADDED '''
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

    ''' ADDED '''
    rospy.Subscriber( 'controller_data', String, controller_callback )
    
    # we obtain the tank service
    rospy.wait_for_service('tank') # wait until the motors are available
    D.tank = rospy.ServiceProxy('tank', Tank) # D.tank is our "driver"
    
    # we obtain the song service
    rospy.wait_for_service('song') # wait until our voice is available
    D.song = rospy.ServiceProxy('song', Song) # D.song is our "speaker" 


''' ADDED '''
####
# Controller input
####

def controller_callback(data):
    global D
    message = data.data
    update_D(D, message)
    # print the PADS and BTNS
    print "PADS is", D.PADS
    print "BTNS is", D.BTNS

    '''
    The intent now: have one standard velocity for both wheels. Up and down
    just change signs of velocities. Left and right add/subtract a certain
    delta. The average velocity of both wheels remains constant. The standard
    velocity can be changed with shoulder buttons.
    '''
    if D.PADS[5] == -1.0: # up
        D.delta = 0
        D.leftVel = math.copysign(D.leftVel, 1)
        D.rightVel = math.copysign(D.rightVel, 1)
        D.tank(D.leftVel, D.rightVel)
    if D.PADS[5] == 1.0: # down
        D.delta = 0
        D.leftVel = math.copysign(D.leftVel, -1)
        D.rightVel = math.copysign(D.rightVel, -1)
        D.tank(-D.leftVel, -D.rightVel)
    if D.PADS[4] == -1.0: # left
        D.delta -= 10
        D.tank(D.leftVel + D.delta, D.rightVel - D.delta)
    if D.PADS[4] == 1.0: # right
        D.delta += 10
        D.tank(D.leftVel + D.delta, D.rightVel - D.delta)

    #if D.BTNS[1] == 1.0: # button 2 sings
    #    D.song([76,76,30,76,30,72,76,30,79], [15,15,15,15,15,15,15,15,15]) # durations
    if D.BTNS[4] == 1.0:
        D.leftVel -= 10
        D.rightVel -= 10
        print 'Steady speed decreased to ', str((D.leftVel + D.rightVel) / 2)
    if D.BTNS[5] == 1.0:
        D.leftVel += 10
        D.rightVel += 10
        print 'Steady speed increased to ', str((D.leftVel + D.rightVel) / 2)
    if D.BTNS[2] == 1.0: # button 3 stops
        D.delta = 0
        D.tank(0,0)



####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
#### 

if __name__ == "__main__":
   main()