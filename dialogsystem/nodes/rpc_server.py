#!/usr/bin/env python
from SimpleXMLRPCServer import  SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
import rospy
import roslib
from std_msgs.msg import String

# Create ros-server-node and rpc-server for Pepper 


class Server:
   def __init__(self):
        rospy.init_node('server')
        self.confirmation="0"
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Starting server node...")
        #read the parameter
        self.IP = rospy.get_param("~SERVER_IP", "127.0.0.1")
        self.PORT = rospy.get_param("~SERVER_PORT", "8000") 
        #Publisher
        self.pub=rospy.Publisher('~status',String,queue_size=1000)
        #rpc server
        rospy.loginfo("Starting rpc server ...")
        self.server = SimpleXMLRPCServer((self.IP, int(self.PORT)))
        self.server.register_function(self.setStatus)
   

   # Set the status of the central system(PR2)
   # status is a string integer
   # status "1" if the work was successful completed
   # status "-1" if the work was unsuccessful
   # return standard string integer for status successful received: "0"
     
   def setStatus(self,status):
       #publish status of PR2
       self.pub.publish(String(status))
       return self.confirmation

   def cleanup(self):
        rospy.loginfo("Shutting down server node...")
        self.server.server_close()

   def run(self):
       while not rospy.is_shutdown():
             self.server.handle_request()
                     
                    

if __name__=="__main__":
    
    try:
        Server().run()
    except:
        rospy.loginfo("Shutting down rpc server...")



