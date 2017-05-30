#!/usr/bin/env python
import nao_speech
import time
import rospy
import sys
from naoqi import (ALBroker, ALProxy, ALModule)
from std_msgs.msg import( String )
from naoqi_bridge_msgs.msg import(
    WordRecognized,
    )

#global variable
sr=None

class Util:
    # Methods for name conversion
    @staticmethod
    def to_naoqi_name(name):
        return "ros{}_{}".format(
            name.replace("/", "_"),
            rospy.Time.now().to_sec() )



class Constants:
    EVENT = ["ALBasicAwareness/HumanTracked","ALBasicAwareness/HumanLost"]
    


class SpeechRecognitionWrapper(ALModule):

    """ROS wrapper for Naoqi speech recognition"""
    def __init__(self):
 
         
        #constants to mark begin and end of discussions
        self.TALKSTART=':TALKSTART'
        self.TALKEND=':TALKEND'

        #start the node
        rospy.init_node('speechRecognizer')

        #parameter reading
        self.PEPPERIP = rospy.get_param("PEPPERIP", "192.168.101.69")
        self.PEPPERPORT = int(rospy.get_param("PEPPERPORT", 9559))
        #self.config=rospy.get_param("CONFIG", "")        
        # Get a (unique) name for naoqi module which is based on the node name
        # and is a valid Python identifier (will be useful later)
        self.naoqi_name = Util.to_naoqi_name( rospy.get_name() )
        
        

        #Start ALBroker (needed by ALModule)
        self.broker = ALBroker(self.naoqi_name + "_broker",
            "0.0.0.0",   # listen to anyone
             0,           # find a free port and use it
            self.PEPPERIP,          # parent broker IP
            self.PEPPERPORT )       # parent broker port
        
        #Init superclass ALModule
        ALModule.__init__( self, self.naoqi_name )

      
        # Start naoqi proxies
        self.memory = ALProxy("ALMemory",self.PEPPERIP,self.PEPPERPORT)
        #self.mic_spk = ALProxy("ALAudioDevice",self.PEPPERIP,self.PEPPERPORT)
        #self.proxy = ALProxy("ALSpeechRecognition",self.PEPPERIP,self.PEPPERPORT)
        self.al =ALProxy("ALAutonomousLife",self.PEPPERIP,self.PEPPERPORT)
        self.al.switchFocus('emptybehavior/behavior_1')
        self.basic_awareness = ALProxy("ALBasicAwareness", self.PEPPERIP,self.PEPPERPORT)
        self.motion = ALProxy("ALMotion", self.PEPPERIP,self.PEPPERPORT)
        
        #lock speech recognizer
        self.is_speech_reco_started = False
        #self.mic_spk.closeAudioInputs()
        #on stop for publisher
        rospy.on_shutdown(self.cleanup)
        #Keep publisher to send word recognized
        self.pub = rospy.Publisher("~word_recognized", String,queue_size=1000 )
        # Subscribe to the recognition topic
        rospy.Subscriber('/rpc_server/recognition', String, self.on_word_recognized);


        #Install global variables needed by Naoqi
        self.install_naoqi_globals()


        #loading the vocabulary
        #rospy.loginfo( 'FILE CONFIG:'+self.config)
        #rospy.loginfo('Loading vocabulary...')
        #with open(self.config) as f:
        #     self.vocabulary= f.readlines()
        #for i in range(len(self.vocabulary)):
        #    self.vocabulary[i]=self.vocabulary[i][:-1]
        
        #kill running modules  
        for i in range(len(Constants.EVENT)):
		#Check no one else is subscribed to this event
		subscribers = self.memory.getSubscribers(Constants.EVENT[i])
		if subscribers:
		    rospy.logwarn(Constants.EVENT[i]+" already in use by another node")
		    for module in subscribers:
		        self.stop(module,Constants.EVENT[i])


        #subscribe to different events
        self.memory.subscribeToEvent(
            Constants.EVENT[0],
            self.naoqi_name,
            self.on_human_detected.func_name)

        self.memory.subscribeToEvent(
             Constants.EVENT[1],
             self.naoqi_name,
             self.on_human_lost.func_name)


       

        #speech controllers
        
    def startSpeechRecognition(self):
	    """ activate the speech recognition when people disappear"""
	    if not self.is_speech_reco_started:
		    self.is_speech_reco_started = True
		   

    def stopSpeechRecognition(self):
	    """ stop speech recognition if human speaker disapper """
	    if self.is_speech_reco_started:
		self.is_speech_reco_started = False
	    

	#Event handlers   

    def on_human_lost(self, key, value, subscriber_id ):
	    """ raised when people disappear"""
	    self.stopSpeechRecognition()
	    rospy.loginfo('Human likely lost')

    def on_human_detected(self, key, value, subscriber_id ):
	    """ raised when people appear"""
	    rospy.loginfo('Human likely detected: '+str(value))
	    if value >= 0:  # found a new person
	       #alert the dialog Manager to synchronize for communication
	       self.pub.publish(String(self.TALKSTART))
	       self.startSpeechRecognition()
	       rospy.loginfo('new Human likely detected')

	       
    def on_word_recognized(self,msg ):
	"""Publish the words recognized by NAO via ROS """
	rospy.loginfo('******speech detected********')
        if self.is_speech_reco_started:
             if(rospy.get_param('busy','1')==0):
                #self.mic_spk.closeAudioInputs()
  	        self.pub.publish(msg)
                rospy.loginfo('ACCEPTED:***********************************'+msg.data)
	        rospy.set_param('busy',1)

    # Install global variables needed for Naoqi callbacks to work
    def install_naoqi_globals(self):
        globals()[self.naoqi_name] = self
        globals()["memory"] = self.memory

    #stop subscribers to a particular events
    
    def stop(self,module,events):
        rospy.loginfo("Unsubscribing '{}' from NAO services".format(
            module))
        try:
            self.memory.unsubscribeToEvent( events, module )
        except RuntimeError:
            rospy.logwarn("Could not unsubscribe from NAO services")
        rospy.loginfo("Shutting down running speech recognizer node...")


    def cleanup(self,module=None):
        if(module==None):
          module=self.naoqi_name
        rospy.loginfo("Unsubscribing '{}' from NAO services".format(
            module))
        try:
            self.memory.unsubscribeToEvent( Constants.EVENT[0], module )
            self.memory.unsubscribeToEvent( Constants.EVENT[1], module )
            #self.memory.unsubscribeToEvent( Constants.EVENT[2], module )
            #self.proxy.unsubscribe(module)
            self.basic_awareness.stopAwareness()
            #self.motion.rest()
            self.al.stopFocus('emptybehavior/behavior_1')
            self.broker.shutdown()
        except RuntimeError:
            rospy.logwarn("Could not unsubscribe from NAO services")
        rospy.loginfo("Shutting down speech recognizer node...")

    	
         



   

if __name__=="__main__":
    
      global sr  
      sr=SpeechRecognitionWrapper()
      #start
      #sr.motion.wakeUp()
      sr.basic_awareness.setEngagementMode("FullyEngaged")
      sr.basic_awareness.setTrackingMode("MoveContextually")
      sr.basic_awareness.startAwareness()
      #loop
      rospy.spin()
      sys.exit(0)




