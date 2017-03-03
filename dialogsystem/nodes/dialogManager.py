#!/usr/bin/env python


import roslib
import rospy
import xmlrpclib

from std_msgs.msg import( String )

from naoqi_bridge_msgs.msg import(
    WordRecognized )
import sys
from naoqi import ALProxy
import numpy as np
import numpy.linalg as la
import numpy.random as rd

class DialogManager:
    def __init__(self):
 
        ################# attributes ###########################
       
        self.contactStatusPr2=-1                           # returned status of PR2-Robot when trying to connect to it. -1 for no connection
        self.questionnaire=[]                              # Input-Output associations      
        self.dictionary=[]                                 # ordered Bag of Words as Basis of the vector space model 
        self.REQUESTFINALFAILED='REQUESTFINALFAILED'       # pr2 could not execute the task successfully after starting it                
        self.UNKNOWNQUESTION='UNKNOWNQUESTION'             # Reaction id for unknown user inputs
        self.UNKNOWNPARAMETER='UNKNOWNPARAMETER'           # Reaction id for unknown service parameters entered by users
        self.REQUESTFAILED='REQUESTFAILED'                 # Reaction id when impossible to provide the service
        self.REQUESTDONE='REQUESTDONE'                     # Reaction id when service completed
        self.PROCESSINGREQUESTLONG='PROCESSINGREQUESTLONG' # Reaction id when a request can be processed and pr2 is busy
        self.PROCESSINGREQUESTSHORT='PROCESSINGREQUESTSHORT' # Reaction id when a request can be processed and pr2 is idle
        self.CUTCAKE='CUTCAKE'                             # Instruction cut cake
        self.TALKSTART=':TALKSTART'                         #constants to mark begin of discussions
        self.TALKEND=':TALKEND'                             #constants to mark end  of discussions
        self.REFLECTION='please let me call the bakery'    #pause mark
        self.pub=""                                        # publisher of output to speech synthesizer
        self.corepub=''                                    # publisher to core dialog manager topics      
        
        ########## Publisher and Subscriber #####################

        rospy.init_node('dialogManager')
        rospy.on_shutdown(self.cleanup)
        #publisher to the synthesizer message topics
        self.pub = rospy.Publisher("synthetiser/message", String ,queue_size=1000)
        # Subscribe to the asr word_recognized topics
        rospy.Subscriber('speechRecognizer/word_recognized', WordRecognized, self.informationRetrieval)
        # Subscribe to the rpc status topics
        rospy.Subscriber('rpc_server/status', String, self.feedback)
        #publisher to the core manager's input topics
        self.corepub = rospy.Publisher("~coreDialogBus", String,queue_size=1000 )
        # Subscribe to the core manager's output topics
        rospy.Subscriber('dialogCoreServerManager/coreDialogBus', String, self.coreOutputProcessor)


        ############## Information Retrieval #################### 

        #filename [restricted language loading]
        
        config=rospy.get_param("CONFIG", "")
        rospy.loginfo( 'FILE CONFIG:'+config)

        #Loading the dictionary
        rospy.loginfo('Loading and preparing dictionary...')
       
	with open(config) as f:
	     content=f.readlines()

	for i in range(len(content)):
	    content[i]=content[i][:-1]
	    self.questionnaire.append(content[i])
	    #srospy.loginfo(content[i])
	  
	dictionary=" "
	for i in range(len(content)):
	    dictionary+=content[i]
	    dictionary+=" "
	#sort the different word in increasing order
	dictionary=np.sort(dictionary.split())
	#remove duplication
	for word in dictionary:
	    if(word not in self.dictionary):
	       self.dictionary.append(word)
	
	

    #compute the vector model of a message
    def computeVector(self,message):
        vec={}
	for word in self.dictionary:
	    vec[word]=0.0
	for word in message:
	    if word in self.dictionary:
	       vec[word]+=1
	return np.array(vec.values(),float)

    #compute the similarity between two messages as the angle between their vector models[0 Pi]
    def similarity(self,vec1,vec2):
	    normVec1=la.norm(vec1)
	    normVec2=la.norm(vec2)
	    if(normVec1==0.0 or normVec2==0.0):
	      return np.pi#max distance
	    else:
	      cosinusTeta=np.dot(vec1,vec2)
              rospy.loginfo(str(cosinusTeta)+' '+str(normVec1)+' '+str(normVec2))
	      return np.arccos(cosinusTeta/(normVec1*normVec2))
 
   
    #extract (odd/strange) words from a text which do not appear in the dictionary
    def extractOddWord(self,text):
        oddWords=[]
        matchedWords=[]
        for i in range(len(text)):
           if text[i] not in self.dictionary:
              oddWords.append(text[i])
           else:
              matchedWords.append(text[i])
        return [oddWords,matchedWords]
            
           
    #retrieve the message content 
    def informationRetrieval(self, word_recognized):
	msg=""
        #recognized words
        words=word_recognized.words
        #reformulate the user input as a string/sentence
        for i in range(len(words)):
            msg+=words[i]
            msg+=" "
        msg=msg[:-1]
        userinput=msg
        #send recognized words/phrase to core dialogue manager for processing
        self.corepub.publish(String(userinput))
	rospy.loginfo(userinput)
               
           
    # Send pr2 reaction to core dialog manager
    def feedback(self, status):
        #rospy.set_param('receiving',1)
        rospy.loginfo(status.data)
        if(status.data=="1"):
          #pr2-robot completed successfully the task 
          msg=self.REQUESTDONE
        else:
          #pr2-robot cannot complete the task after starting it
          msg=self.REQUESTFINALFAILED
        self.corepub.publish(msg)
	rospy.loginfo(msg)
    
    # Send a request to PR2
    def callPr2(self):
        try:
           pepper = xmlrpclib.ServerProxy('http://'+str(rospy.get_param('PR2IP','127.0.0.1'))+':'+str(rospy.get_param('PR2PORT','8000')))
           self.contactStatusPr2=pepper.cutCake()
        except:
           self.contactStatusPr2=-1
        
    #on close
    def cleanup(self):
        rospy.loginfo("Shutting down dialogManager node...")

    #process output of core dialogue manager
    def coreOutputProcessor(self,msg):
        #a service has been requested
        if(msg.data==self.CUTCAKE):
           self.pub.publish(String(self.REFLECTION))
           self.callPr2()
           if(self.contactStatusPr2<0):
              #call failed
              self.corepub.publish(String(self.REQUESTFAILED))
           else: 
               if(self.contactStatusPr2==0):
                  #call succeeded  and pr2 is free
                  self.corepub.publish(String(self.PROCESSINGREQUESTSHORT))
               else:
                  #call succeeded and pr2 is busy
                  self.corepub.publish(String(self.PROCESSINGREQUESTLONG))
        else:
             #simple output: send the output to synthesizer for speaking
             self.pub.publish(msg)
             

if __name__=="__main__":
    
    try:
        DialogManager()
        rospy.spin()
    except:
        pass
