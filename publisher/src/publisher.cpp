#include "ros/ros.h"
#include <suturo_perception_msgs/ObjectDetection.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void subscriber(const suturo_perception_msgs::ObjectDetection& msg)
{
  geometry_msgs::PoseStamped pose;

  pose = msg.pose;

  tf::TransformBroadcaster br;
  //tf::Transform transform;

  tf::Stamped<tf::Pose> transform;
  //tf2::convert

  tf::poseStampedMsgToTF(pose, transform);

  //ros::Rate rate(10.0);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_mount_kinect_rgb_optical_frame", msg.name));
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("percepteros/object_detection", 1000, subscriber);

  ros::spin();

  return 0;
}

