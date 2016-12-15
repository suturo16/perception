#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String
import sys
from naoqi import ALProxy


class Synthetiser:
    def __init__(self):
        rospy.init_node('synthetiser')
        rospy.on_shutdown(self.cleanup)
        #read the parameter
        self.IP = rospy.get_param("~IP", "192.168.101.69")
        self.PORT = rospy.get_param("~PORT", "9559") 
        # Subscribe to the message topics
        rospy.Subscriber('~message', String, self.synthetise)

       
    #synthesize msg to voice   
    def synthetise(self, msg):
	try:
	    tts = ALProxy("ALTextToSpeech", self.IP, int(self.PORT))
	except Exception,e:
	    rospy.loginfo("Could not create proxy to ALTextToSpeech")
	    rospy.loginfo("Error was: ",e)
	    return 1
        #speak
        rospy.set_param('busy',1)
	tts.say(msg.data)
        # Print the recognized words on the screen
        rospy.loginfo(msg.data)
        rospy.sleep(2)
        rospy.set_param('busy',0)

    def cleanup(self):
        rospy.loginfo("Shutting down synthetiser node...")

if __name__=="__main__":
    
    try:
        Synthetiser()
        rospy.spin()
    except:
        pass
