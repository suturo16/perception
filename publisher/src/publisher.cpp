#include "ros/ros.h"
#include <suturo_perception_msgs/ObjectDetection.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

ros::Publisher vis_pub1;
ros::Publisher vis_pub2;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void subscriber(const suturo_perception_msgs::ObjectDetection& msg)
{
    if(msg.type==2){
      geometry_msgs::PoseStamped pose;

      pose = msg.pose;

      tf::TransformBroadcaster br;
      //tf::Transform transform;

      tf::Stamped<tf::Pose> transform;
      //tf2::convert

      tf::poseStampedMsgToTF(pose, transform);

      //ros::Rate rate(10.0);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_mount_kinect_rgb_optical_frame", msg.name));

      visualization_msgs::Marker marker;
      marker.header = pose.header;
      marker.ns = "percepteros";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = pose.pose;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      std::cout<<"Publishing";

      visualization_msgs::Marker markerCyl;
      markerCyl.header = pose.header;
      markerCyl.id=1;
      markerCyl.type = visualization_msgs::Marker::CYLINDER;
      markerCyl.ns = "percepteros";
      markerCyl.pose = pose.pose;
      markerCyl.scale.x = msg.width;
      markerCyl.scale.y = msg.depth;
      markerCyl.scale.z = msg.height;
      // Set the color -- be sure to set alpha to something non-zero!
      markerCyl.color.r = 1.0f;
      markerCyl.color.g = 1.0f;
      markerCyl.color.b = 0.0f;
      markerCyl.color.a = 1.0;


      vis_pub1.publish(marker);
      vis_pub2.publish(markerCyl);
    }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("percepteros/object_detection", 1000, subscriber);

  vis_pub1= n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
  vis_pub2= n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );

  ros::spin();

  return 0;
}

