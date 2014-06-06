#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
from std_msgs.msg import String


####
# echo_subscriber.py ~ showing off ROS subscribing...
####


def main():
    """ establishes our ROS node
        subscribes to the published stream (at stream_name)
    """
    rospy.init_node('listener', anonymous=True)  # usual node-naming

    # the key piece of information is the name of the published stream
    stream_name = 'text_data'

    # this subscribes to the stream
    # (1) it names the stream (stream_name)
    # (2) it indicates the type of message received (String)
    # (3) it indicates a function (callback) to be called w/ each message
    rospy.Subscriber( stream_name, String, callback )

    # wait until rospy.signal_shutdown
    rospy.spin()  



####
# callback ~ called with each published message
####
def callback(data):
    """ This function is called for each published message
    """
    message = data.data
    print "I received the string", message
    
    # if the message is the string 'q', we shutdown
    if message == 'q':
        rospy.signal_shutdown("Quit requested.")
        


####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
####

if __name__ == '__main__':
    main()