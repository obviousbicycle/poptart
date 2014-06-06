#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import sensor_msgs.msg as sm
from std_msgs.msg import String


####
# text_publisher.py ~ showing off ROS publishing...
####


####
# main
####

def main():
    """ the main function that sets everything up
    """
    # Initialize this ROS node
    rospy.init_node('text_publisher') # any name will do

    # create a String publisher (pub)
    # you will be able to subscribe to it via the name 'text_data'
    pub = rospy.Publisher('text_data',String)
    
    # main loop
    # here, a person can type messages, which will be published
    while rospy.is_shutdown() == False: # standard infinite loop in ROS

        print "Enter a message to publish: ",
        user_message = raw_input()      # blocking input of a Python string
        print "Publishing", user_message
        pub.publish(String(user_message)) # converts to ROS string first...

        if user_message == 'q':   # quit on an input of 'q'
            break  

    print "Bye!"
    return




####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
####

if __name__ == "__main__":
    main()