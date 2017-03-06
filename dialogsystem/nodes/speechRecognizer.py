#!/usr/bin/env python

import rospy
import sys
from naoqi import (ALBroker, ALProxy, ALModule)
from std_msgs.msg import( String )
from naoqi_bridge_msgs.msg import(
    WordRecognized,
    )


class Util:
    # Methods for name conversion
    @staticmethod
    def to_naoqi_name(name):
        return "ros{}_{}".format(
            name.replace("/", "_"),
            rospy.Time.now().to_sec() )



class Constants:
    EVENT = "WordRecognized"
    


class SpeechRecognitionWrapper(ALModule):

    """ROS wrapper for Naoqi speech recognition"""
    def __init__(self):

        #start the node
        rospy.init_node('speechRecognizer')

        #parameter reading
        self.ip = rospy.get_param("~IP", "~192.168.101.69")
        self.port = int(rospy.get_param("PORT", 9559))
        self.config=rospy.get_param("CONFIG", "")
        rospy.loginfo( 'FILE CONFIG:'+self.config)
        # Get a (unique) name for naoqi module which is based on the node name
        # and is a valid Python identifier (will be useful later)
        self.naoqi_name = Util.to_naoqi_name( rospy.get_name() )

        #Start ALBroker (needed by ALModule)
        self.broker = ALBroker(self.naoqi_name + "_broker",
            "0.0.0.0",   # listen to anyone
             0,           # find a free port and use it
            self.ip,          # parent broker IP
            self.port )       # parent broker port


        #Init superclass ALModule
        ALModule.__init__( self, self.naoqi_name )

        # Start naoqi proxies
        self.memory = ALProxy("ALMemory")
        self.proxy = ALProxy("ALSpeechRecognition",self.ip,self.port)
        

        #on stop
        rospy.on_shutdown(self.stop)
        #Keep publisher to send word recognized
        self.pub = rospy.Publisher("~word_recognized", WordRecognized,queue_size=1000 )

        #Install global variables needed by Naoqi
        self.install_naoqi_globals()

        #Check no one else is subscribed to this event
        subscribers = self.memory.getSubscribers(Constants.EVENT)
        if subscribers:
            rospy.logwarn("Speech recognition already in use by another node")
            for module in subscribers:
                self.stop(module)

        # Configure this instance
        self.proxy.pause(1)
        self.proxy.setLanguage('English')
        #loading the vocabulary
        dialog=[]
        rospy.loginfo('Loading vocabulary...')
        with open(self.config) as f:
             content = f.readlines()
        for i in range(len(content)):
            content[i]=content[i][:-1]
            dialog.append(content[i])
            rospy.loginfo(content[i])
        while('?' in content):
             content.remove('?')
        #rospy.loginfo(dialog)
        try:
          self.proxy.setVocabulary(content,False)
        except:
          pass
        self.proxy.setVisualExpression(True)
        self.proxy.pause(0)

        #subscribing to recognition

        self.proxy.subscribe(self.naoqi_name)

        #And subscribe to the event raised by speech recognition
        rospy.loginfo("Subscribing '{}' to NAO speech recognition".format(
            self.naoqi_name) )
        rospy.loginfo("TESTING")

        self.memory.subscribeToEvent(
            Constants.EVENT,
            self.naoqi_name,
            self.on_word_recognised.func_name )


    # Install global variables needed for Naoqi callbacks to work
    def install_naoqi_globals(self):
        globals()[self.naoqi_name] = self
        globals()["memory"] = self.memory


    def stop(self,module=None):
        if module is None:
           module = self.naoqi_name
        rospy.loginfo("Unsubscribing '{}' from NAO speech recognition".format(
            module))
        try:
            self.memory.unsubscribeToEvent( Constants.EVENT, module )
            self.proxy.unsubscribe(module)
        except RuntimeError:
            rospy.logwarn("Could not unsubscribe from NAO speech recognition")
        rospy.loginfo("Shutting down speech recognizer node...")

    def on_word_recognised(self, key, value, subscriber_id ):
        """Publish the words recognized by NAO via ROS """
        rospy.loginfo('BUSY:='+str(rospy.get_param('busy','1')))
        if(rospy.get_param('busy','1')==0):
                #rospy.set_param('busy',1)
		#Create dictionary, by grouping into tuples the list in value
		temp_dict = dict( value[i:i+2] for i in range(0, len(value), 2) )

		#Delete empty string from dictionary
		#temp_dict={4:5}
		rospy.loginfo("Detection:"+str(temp_dict))
		self.pub.publish(WordRecognized( temp_dict.keys(), temp_dict.values() ))
		
         



   

if __name__=="__main__":
    
  
      sr=SpeechRecognitionWrapper()
      #loop
      rospy.spin()
  
      sys.exit(0)




