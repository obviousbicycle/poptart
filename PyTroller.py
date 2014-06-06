# by Andrew Fishberg et al. Spring 2014

import sys
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import sensor_msgs.msg as sm
from std_msgs.msg import String
#
def toHexString(hex):
   return "0x" + "".join(reversed(hex[4:]))

def main():
    """ the main function that sets everything up
    """
    # Initialize this ROS node
    rospy.init_node('controller_publisher') # any name will do

    # create a String publisher (pub)
    # you will be able to subscribe to it via the name 'text_data'
    pub = rospy.Publisher('controller_data', String)
    pipe = open('/dev/input/js0', 'r')
    action = []

    # main loop
    # here, a person can type messages, which will be published
    while rospy.is_shutdown() == False: # standard infinite loop in ROS
       #print "Loop"
       for character in pipe.read(1):
          #print "Loop"
          action += ['%02X' % ord(character)]
          if len(action) == 8:
            hstr = toHexString(action)
            print "Publishing", hstr
            pub.publish(String(hstr))
            # converts to ROS string first...
            action = []

####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
####

if __name__ == "__main__":
    main()