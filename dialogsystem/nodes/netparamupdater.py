#!/usr/bin/env python
import xmlrpclib
import rospy
import roslib
import socket
import fcntl
import struct
from std_msgs.msg import String

# Create Network parameter updater


class Updater:
   def __init__(self):
        rospy.init_node('netparamupdater')
        self.confirmation="0"
        rospy.on_shutdown(self.cleanup)
        self.RPCSERVERIP = rospy.get_param("RPCSERVERIP", "127.0.0.1")
        self.RPCSERVERPORT = str(rospy.get_param("RPCSERVERPORT", "8000")) 
        rospy.loginfo("Starting netparamupdater node...")
        
        
   def cleanup(self):
        rospy.loginfo("Shutting down netparamupdater node...")
 
   
   def getIP(self):
	    self.SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.INTERFACE = rospy.get_param("INTERFACE", "wlan0")
	    self.ADDR=str(socket.inet_ntoa(fcntl.ioctl(self.SOCKET.fileno(), 0x8915, struct.pack('256s', self.INTERFACE[:15]))[20:24]))
            self.SOCKET.close()
            return str(self.ADDR)

   def getPORT(self):
       return str(rospy.get_param("RPCSERVERPORT", "8000"))
   
   
	   
   def test(self):
       #rospy.set_param("INTERFACE", "welan0")
       rospy.loginfo('PARAM:'+rospy.get_param('INTERFACE','elannn'))

   def run(self):
       while not rospy.is_shutdown():          
	 try:
           port=self.getPORT()
           ip=self.getIP()
           
           if((self.RPCSERVERPORT!=port) or (self.RPCSERVERIP!=ip)):
               self.RPCSERVERPORT=port
               self.RPCSERVERIP=ip
               rospy.set_param("RPCSERVERIP", self.RPCSERVERIP)
               rospy.set_param("RPCSERVERPORT", self.RPCSERVERPORT) 
	       self.pr2 = xmlrpclib.ServerProxy('http://'+str(self.RPCSERVERIP)+':'+str(self.RPCSERVERPORT))
               clientID=0 #about pepper-robot's parameters
	       self.pr2.updateObserverClient(self,clientID,self.RPCSERVERIP, self.RPCSERVERPORT)
	 except:
	   pass
        
         rospy.loginfo('The new addresses are:'+self.RPCSERVERPORT+' and '+self.RPCSERVERIP)
         rospy.loginfo('The old addresses are:'+port+' and '+ip)
         rospy.sleep(5) #sleep 10s
      
                     
                                   











if __name__=="__main__":
    
    try:
        #Updater().test()
         Updater().run()
    except:
        rospy.loginfo("Shutting down netparamupdater...")

