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
        self.RPCSERVERIP = rospy.get_param("RPCSERVERIP", "127.0.0.1")
        self.RPCSERVERPORT = rospy.get_param("RPCSERVERPORT", "8000") 
        #Publisher
        self.pub=rospy.Publisher('~status',String,queue_size=1000)
        #rpc server
        rospy.loginfo("Starting rpc server ...")
	#make sure the right address ip is stored in launch file, otherwise it failed
        self.server = SimpleXMLRPCServer((self.RPCSERVERIP, int(self.RPCSERVERPORT)))
        self.server.register_function(self.notify)
        self.server.register_function(self.updateObserverClient)

   # notify the completion of the work by the central system(PR2)
   # status is a string integer
   # status "1" if the work was successful completed: default
   # status "-1" if the work was unsuccessful
   # return standard string integer for status successful received: "0" defaullt, -1 otherwise
     
   def notify(self):
       #publish status of PR2
       status=1
       self.pub.publish(String(str(status)))
       return self.confirmation


   #update connection parameters
   #clientID is the ID of the machine, whose parameters changed(pepper=0, pr2=1 and turtle=2)
   
   def updateObserverClient(self,clientID, host, port):
       #for the moment I am only interested in pr2ID
       if(clientID==1):
         if(str(rospy.get_param('PR2IP','127.0.0.1'))!=str(host)):
            #update
            rospy.set_param('PR2IP',str(host))

         if(str(rospy.get_param('PR2PORT','8000'))!=str(port)):
            #update
            rospy.set_param('PR2PORT',str(port))
        
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


