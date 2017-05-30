#!/usr/bin/env python
import paramiko
import rospy
import roslib
from std_msgs.msg import String
import utility as u
import  paramiko.client as pc
# Create ros-server-node and rpc-server for Pepper 



class GstSphinx:
   def __init__(self):
      try:
        rospy.init_node('gstreamer_sphinx')
        self.client=None
        self.confirmation="0"
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Starting gstreamer streaming...")
        #read the parameter
        self.RHOST = rospy.get_param("RHOST", "192.168.101.69")
        self.RUSERNAME = rospy.get_param("RUSERNAME", "nao")
        self.RPORT = rospy.get_param("RPORT", "22") 
        self.RPASSWORD= rospy.get_param("RPASSWORD", "iai") 
        self.HOST = rospy.get_param("HOST", "localhost")
        self.PORT = rospy.get_param("PORT", "7000")
        #client
        rospy.loginfo("load parameters...")
        self.client=pc.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.load_system_host_keys()
        rospy.loginfo("instantiate client...")
        self.client.connect(self.RHOST,username=self.RUSERNAME,port=int(self.RPORT), password=self.RPASSWORD)
        rospy.loginfo("connect to remote host...")
        self.command="gst-launch-0.10 pulsesrc "+" ! 'audio/x-raw-int, format=S16LE, channels=1, width=16,                          depth=16,rate=16000' ! "+"tcpclientsink port="+str(self.PORT)+" host="+str(self.HOST)
        #wait until gstreamer-server started before starting the client
        while int(rospy.get_param("ORDER", "0"))!=1:
              rospy.loginfo("loop ...")
              rospy.sleep(10) 
        self.stdin, self.stdout, self.stderr =self.client.exec_command(self.command)
        rospy.loginfo("execute command...")
        rospy.loginfo("output: "+str(self.stdout.read()))
        rospy.loginfo("streaming...")
      except:
        if(self.client !=None):
           self.client.close()

   def cleanup(self):
       if(self.client !=None):
           self.client.close()
       rospy.loginfo("Shutting down GstSphinx: streaming client...")
      
   
if __name__=="__main__":
   
     GstSphinx()
       
       
   
