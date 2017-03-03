#!/usr/bin/env python

import socket
import sys
import rospy
import roslib
from std_msgs.msg import String
import subprocess
#Here is a client for core management of dialogue


SELF_CLIENT_SOCKET=''

class DialogCoreClientManager(object):
   def __init__(self):

        #constants to mark begin and end of discussions
        self.TALKSTART=':TALKSTART'
        self.TALKEND=':TALKEND'
        self.INIT   =':INIT'
        self.USERID=0
        rospy.init_node('dialogCoreServerManager')
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Starting core server manager node...")
        #read the parameter
        self.CORESERVERIP = rospy.get_param("CORESERVERIP", "localhost")
        self.CORESERVERPORT = int(rospy.get_param("CORESERVERPORT", "1024"))
        self.CORESERVERPATH=rospy.get_param("CORESERVERPATH", "clear")
        self.CORESERVERCWD=rospy.get_param("CORESERVERCWD", "")
        self.ADDR = (self.CORESERVERIP, self.CORESERVERPORT) 
        self.USERNAME= 'user'+str(self.USERID)
        self.BOT=''
        self.DATA_IN=''
        self.DATA_OUT=''
        #Publisher
        self.pub=rospy.Publisher('~coreDialogBus',String,queue_size=1000)
        # Subscriber
        rospy.Subscriber('dialogManager/coreDialogBus', String, self.process)
        #chartscript server
        rospy.loginfo("starting core server Manager ...")
        self.SERVERPROCESS=subprocess.Popen(self.CORESERVERPATH,cwd=self.CORESERVERCWD,shell=True)    
        #connection to chatscript server cd BINARIES && ./LinuxChatScript64
        #initalize the dialogue
        #rospy.sleep(10)
        #self.process(String(self.TALKSTART))

  

   def cleanup(self):
        global SELF_CLIENT_SOCKET
        rospy.loginfo("Shutting down core server manager node...")
        self.SERVERPROCESS.terminate()
        self.SERVERPROCESS.kill()
        SELF_CLIENT_SOCKET.close()

   def process(self,msg):
	  global SELF_CLIENT_SOCKET
          #connection to chatscript server cd BINARIES && ./LinuxChatScript64
          try:
		  SELF_CLIENT_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		  SELF_CLIENT_SOCKET.connect(self.ADDR)
	  
                  
		  #read request of the user
		 
		  if(msg.data==self.TALKSTART):
			 self.DATA_IN=':reset'
		  else:
			 self.DATA_IN=msg.data

		  #request of the user sent to for processing
		  SELF_CLIENT_SOCKET.sendall((self.USERNAME+chr(0)+self.BOT+chr(0)+self.DATA_IN +chr(0))) 

		  #system's reaction
	 	  self.DATA_OUT = SELF_CLIENT_SOCKET.recv(80)
		  
		  #publish reaction
                  self.pub.publish(String(self.DATA_OUT))

                  #change user
		  if(msg.data==self.TALKSTART):
                    self.USERID+=1
                    self.USERNAME= 'user'+str(self.USERID)
		  
                   

                  #print Inputs and Outputs of core system manager

                  rospy.loginfo('CORE SERVER INPUT: '+self.DATA_IN) 
		  rospy.loginfo('CORE SERVER OUTPUT: '+self.DATA_OUT) 

		  #close socket allocated for request 
		  SELF_CLIENT_SOCKET.close() 
          except:
                  rospy.loginfo('Error occured!!!'+str(self.ADDR))  
		  

	 
	  
 
  
                    

if __name__=="__main__":
    
    try:
        DialogCoreClientManager()
        rospy.spin()
    except:
        rospy.loginfo("core server Manager shut down ...")
