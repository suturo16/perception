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
        self.QUESTIONID=-1
        self.isFormulatingRequest=False
        self.serviceID=[0]
        self.serviceParameterID=[[1]]
        self.contactStatusPr2=-1
        self.serviceStatusPr2=-1

        rospy.init_node('dialogManager')
        rospy.on_shutdown(self.cleanup)
        self.config=rospy.get_param("CONFIG", "")
        rospy.loginfo( 'FILE CONFIG:'+self.config)
        #publisher to the message topics
        self.pub = rospy.Publisher("synthetiser/message", String )
        # Subscribe to the word_recognized topics
        rospy.Subscriber('speechRecognizer/word_recognized', WordRecognized, self.informationRetrieval)
         # Subscribe to the status topics
        rospy.Subscriber('rpc_server/status', String, self.feedback)


         #Loading the dictionary
        rospy.loginfo('Loading and preparing dictionary...')
        with open(self.config) as f:
             content = f.readlines()
        
        dialog=[]
        for i in range(len(content)):
            content[i]=content[i][:-1]
            dialog.append(content[i])
            rospy.loginfo(content[i])
        
        while('?' in content):
             content.remove('?')
        
        #preparing the dictionary
        dictionary=" "
        for i in range(len(content)):
            dictionary+=content[i]
            dictionary+=" "
        #sort the different word in increasing order
        dictionary=np.sort(dictionary.split())
        #remove duplication
        self.dictionary=[]
        for word in dictionary:
            if(word not in self.dictionary):
               self.dictionary.append(word)
        
        #many-to-many association between answersand 
        #? separates answers from questions
        i=1 #read the first #

        self.questionnaire=[]
	while(i<len(dialog)):
	     answer=[]
	     question=[]
	     while(i<len(dialog) and dialog[i]!='?'):
		  question.append(dialog[i])
		  i+=1
	     i+=1
	     while(i<len(dialog) and dialog[i]!='?'):
		  answer.append(dialog[i])
		  i+=1
             i+=1
             self.questionnaire.append([list(question),list(answer)])

        #some key-parameter
        self.UNKNOW_QUESTION=len(self.questionnaire)-1
        self.UNKNOWN_PARAMETER=self.UNKNOW_QUESTION-1
        self.REQUEST_FAILED=self.UNKNOWN_PARAMETER-1
        self.REQUEST_DONE=self.REQUEST_FAILED-1
        self.PROCESSING_REQUEST=self.REQUEST_DONE-1

    #compute the vector model of a message
    def computeVector(self,message):
        vec={}
	for word in self.dictionary:
	    vec[word]=0.0
	for word in message:
	    if word in self.dictionary:
	       vec[word]+=1
	return np.array(vec.values(),float)

    #compute the similarity between two message as the angle between their vector models[0 Pi]
    def similarity(self,vec1,vec2):
	    normVec1=la.norm(vec1)
	    normVec2=la.norm(vec2)
	    if(normVec1==0.0 or normVec2==0.0):
	      return np.pi#max distance
	    else:
	      cosinusTeta=np.dot(vec1,vec2)
	      return np.arccos(cosinusTeta/(normVec1*normVec2))

    #retrieve the message content 
    def informationRetrieval(self, word_recognized):
	msg=""
        words=word_recognized.words
        for i in range(len(words)):
            msg+=words[i]
            msg+=" "
        #compute the most probable message
        minindex=-1
        minteta=np.pi
        for i in range(len(self.questionnaire)):
           
            answer=self.questionnaire[i][0]
            for j in range(len(answer)):
                distance=self.similarity(self.computeVector(msg.split()),self.computeVector(answer[j].split()))
                #rospy.loginfo('STATE:'+str(distance)+msg+answer[j])
                if(distance<=minteta):
                   minteta=distance
                   minindex=i
        #Analysing the state of the dialogue: querying/becoming answer/answering...  
        isFormulatingRequest=False#default status
        if(minindex==-1 and self.isFormulatingRequest):#unknown parameter
           minindex=self.UNKNOWN_PARAMETER
           self.isFormulatingRequest=False
           isFormulatingRequest=False

        if(minindex==-1 and (not self.isFormulatingRequest)):#unknown question
           minindex=self.UNKNOWN_QUESTION
           isFormulatingRequest=False

        if(minindex!=-1 and self.isFormulatingRequest and (minindex not in self.serviceParameterID[self.QUESTIONID])):#unknown parameter
           minindex=self.UNKNOWN_PARAMETER
           self.isFormulatingRequest=False
           isFormulatingRequest=False
           self.QUESTIONID=-1

        if((not self.isFormulatingRequest) and (minindex in self.serviceID)):
          #changing the state of pepper: a new request detected
           isFormulatingRequest=True
           self.QUESTIONID=minindex

        if(len(self.questionnaire[minindex][1])==0):
           msg=""
        else:
           msg=self.questionnaire[minindex][1][rd.randint(len(self.questionnaire[minindex][1]))]
        #publish the message to be said
        self.pub.publish(msg)
        rospy.loginfo(msg)
        if(self.isFormulatingRequest):
          #send request to PR2-Robot
	  self.callPr2()
	  if(self.contactStatusPr2=='1'):
             minindex=self.PROCESSING_REQUEST
          else:
	     minindex=self.REQUEST_FAILED
	  
          if(len(self.questionnaire[minindex][1])==0):
             msg=""
          else:
             msg=self.questionnaire[minindex][1][rd.randint(len(self.questionnaire[minindex][1]))]
          #publish  first feedback from PR2
	  self.pub.publish(msg)
	  rospy.loginfo(msg)
	  #reset the status of PR2
	  self.contactStatusPr2='-1'
          #reset the status of Pepper
        self.isFormulatingRequest=isFormulatingRequest
    
    def feedback(self, status):
        #rospy.set_param('receiving',1)
        rospy.loginfo(status.data)
        if(status.data=="1"):
          msg=self.questionnaire[ self.REQUEST_DONE][1][rd.randint(len(self.questionnaire[ self.REQUEST_DONE][1]))]
        else:
          msg=self.questionnaire[self.REQUEST_FAILED][1][rd.randint(len(self.questionnaire[self.REQUEST_FAILED][1]))]
        self.pub.publish(msg)
	rospy.loginfo(msg)

    def callPr2(self):
        try:
           pepper = xmlrpclib.ServerProxy('http://'+str(rospy.get_param('~PR2_IP','127.0.0.1'))+':'+str(rospy.get_param('~PR2_PORT','8000')))
           self.contactStatusPr2=pepper.getCake()
        except:
           self.contactStatusPr2=-1
        

    def cleanup(self):
        rospy.loginfo("Shutting down dialogManager node...")

if __name__=="__main__":
    
    try:
        DialogManager()
        rospy.spin()
    except:
        pass
