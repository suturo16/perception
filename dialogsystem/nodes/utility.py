#!/usr/bin/env python
import rospy
import roslib
import numpy as np
import numpy.linalg as la
import random as rd

############## Information Retrieval #################### 

        #filename [restricted language loading]

class Utility(object):
        
    def __init__(self, filename):
        self.filename=filename
        self.dictionary=[]
        self.vocabulary=[]
        self.dataset=[]
       

    def parse(self):
        #Loading the dictionary
        rospy.loginfo('Loading and parsing dictionary...')
        config=self.filename
	with open(config) as f:
	     content=f.readlines()

	for i in range(len(content)):
	    content[i]=content[i][:-1]
	    self.vocabulary.append(content[i].upper())
            self.dataset.append(content[i])
	  
	dictionary=" "
	for i in range(len(content)):
	    dictionary+=content[i].upper()
	    dictionary+=" "
	#sort the different word in increasing order
	dictionary=np.sort(dictionary.split())
	#remove duplication
	for i in range(len(dictionary)):
            word=dictionary[i]
	    if(word not in self.dictionary):
	       self.dictionary.append(word)
	
	

    #compute the vector model of a message
    def computeVector(self,message):
        vec={}
	for i in range(len(self.dictionary)):
            word=self.dictionary[i]
	    vec[word]=0.0
	for i in range(len(message)):
            word=message[i]
	    if word in self.dictionary:
	       vec[word]+=1
        vect=[]
        for i in range(len(self.dictionary)):
            word=self.dictionary[i]
            vect.append(vec[word])
	return np.array(vect,float)

    #compute the similarity between two messages as the angle between their vector models[0 Pi]
    def similarity(self,vec1,vec2):
	    normVec1=la.norm(vec1)
	    normVec2=la.norm(vec2)
	    if(normVec1==0.0 or normVec2==0.0):
	      return np.pi#max distance
	    else:
              if list(vec1)==list(vec2):
                 return 0.0
              else:
		 cosinusTeta=np.dot(vec1,vec2)
		 rospy.loginfo(str(cosinusTeta)+' '+str(normVec1)+' '+str(normVec2))
		 return np.arccos(cosinusTeta/(normVec1*normVec2))

    #retrieve the message content 
    def informationRetrieval(self,wordRecognised):
	minindex=0
        minvalue=self.similarity(self.computeVector(wordRecognised.split()),self.computeVector(self.vocabulary[minindex].split()))
        #recognized words
        #search best imput
        for i in range(len(self.vocabulary)):
            value=self.similarity(self.computeVector(wordRecognised.split()),self.computeVector(self.vocabulary[i].split()))
            if(value<minvalue):
              minvalue=value
              minindex=i
        return self.dataset[minindex]

    #generate corpora
    def gencorpora(self,nfiles, fsize):        
        data=[]
        sample=[]
        for i in range(nfiles):
            data.append([])
        for i in range(fsize):
            for j in range(nfiles):
                if len(sample)==0:
                   for k in range(len(self.dataset)):
                       sample.append(self.dataset[k])
                k=rd.randint(0,len(sample)-1)
                data[j].append(sample[k])
        filename='pepper'
        k=30
        for i in range(nfiles):
            np.savetxt(filename+str(k+i)+'.txt',data[i],fmt='%s')
                



