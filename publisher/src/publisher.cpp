#include "ros/ros.h"
#include <suturo_perception_msgs/ObjectDetection.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include <iostream>

ros::Publisher vis_pub1;
ros::Publisher vis_pub2;
tf::TransformBroadcaster* br;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void subscriber(const suturo_perception_msgs::ObjectDetection& msg)
{
    if(msg.type==6){
      geometry_msgs::PoseStamped pose;

      pose = msg.pose;

      //tf::Transform transform;

      tf::Stamped<tf::Pose> transform;
      //tf2::convert

      tf::poseStampedMsgToTF(pose, transform);

      br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), pose.header.frame_id, msg.name));

      visualization_msgs::Marker marker;
      marker.header = pose.header;
      marker.ns = "percepteros";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = pose.pose;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      std::cout<<"Publishing";

      vis_pub1.publish(marker);
    } /*else if (msg.type==5) {
    	geometry_msgs::PoseStamped pose;

      	pose = msg.pose;

      	tf::TransformBroadcaster br;

      	tf::Stamped<tf::Pose> transform;

      	tf::poseStampedMsgToTF(pose, transform);

      	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_mount_kinect_rgb_optical_frame", msg.name));

     	visualization_msgs::Marker markerTray;
      	markerTray.header = pose.header;
      	markerTray.id=2;
      	markerTray.type = visualization_msgs::Marker::CUBE;
      	markerTray.ns = "percepteros";
      	markerTray.pose = pose.pose;
      	markerTray.scale.x = msg.width;
      	markerTray.scale.y = msg.depth;
      	markerTray.scale.z = 0.1;
      	// Set the color -- be sure to set alpha to something non-zero!
      	markerTray.color.r = 1.0f;
      	markerTray.color.g = 1.0f;
      	markerTray.color.b = 0.0f;
      	markerTray.color.a = 1.0;

      	vis_pub2.publish(markerTray);
    }*/
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  tf::TransformBroadcaster bro;
  br = &bro;

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("percepteros/object_detection", 1000, subscriber);

  vis_pub1= n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  vis_pub2= n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  ros::spin();

  return 0;
}

