// UIMA
#include <uima/api.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <percepteros/types/all_types.h>
#include <suturo_perception_msgs/ObjectDetection.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>

using namespace uima;

class ROSPublisher : public DrawingAnnotator
{
private:

  ros::NodeHandle n;
  ros::Publisher chatter_pub;
  ros::Publisher debug_pub;

  tf::TransformListener listener;

  tf::StampedTransform camToWorld, worldToCam;
  int currentid = 0;


public:
  ROSPublisher() : DrawingAnnotator(__func__)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    int argc=0;
    char* argv[0];
    ros::init(argc, argv, "detected_objects");
    chatter_pub = n.advertise<suturo_perception_msgs::ObjectDetection>("percepteros/object_detection", 1000);
    debug_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

private:
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    camToWorld.setIdentity();
    if(scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outInfo("No camera to world transformation!!!");
    }
    worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);
    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(camToWorld, eigenTransform);

    tf::StampedTransform kinectToOdom;
    Eigen::Affine3d kinectToOdomEigen;

     for(rs::Cluster c: clusters){
        std::vector<percepteros::RecognitionObject> objects;
        std::vector<percepteros::EnclosingCircle> circles;
        std::vector<rs::PoseAnnotation> poses;
        
        c.annotations.filter(objects);
        c.annotations.filter(poses);
        c.annotations.filter(circles);
        for(auto circle: circles){
            publishDebugMarker(circle);
        }
        
        if(objects.size()!=0 && objects.size() == poses.size()){
            for(int i = 0; i < objects.size(); i++){
            	percepteros::RecognitionObject recObj  = objects[i];
                rs::StampedPose pose = poses[i].camera.get();
            	std::vector<double> translation = pose.translation.get();
            	std::vector<double> rotation = pose.rotation.get();

            	Eigen::Matrix3d mat;
            	mat << 	rotation[0], rotation[1], rotation[2], 
            			rotation[3], rotation[4], rotation[5],
                        rotation[6], rotation[7], rotation[8];
                Eigen::Vector3d trans(translation[0],translation[1],translation[2]);
                Eigen::Quaterniond q(mat);
				q.normalize();
            	
                outInfo(recObj.name.get());

                suturo_perception_msgs::ObjectDetection objectDetectionMsg;
                
                objectDetectionMsg.pose.header.frame_id = "/head_mount_kinect_rgb_optical_frame";

                objectDetectionMsg.pose.pose.position.x=trans[0];
                objectDetectionMsg.pose.pose.position.y=trans[1];
                objectDetectionMsg.pose.pose.position.z=trans[2];
                
                objectDetectionMsg.pose.pose.orientation.x=q.x();
                objectDetectionMsg.pose.pose.orientation.y=q.y();
                objectDetectionMsg.pose.pose.orientation.z=q.z();
                objectDetectionMsg.pose.pose.orientation.w=q.w();
				
				geometry_msgs::PoseStamped nPose;
				listener.transformPose("/odom_combined", objectDetectionMsg.pose, nPose);
				objectDetectionMsg.pose = nPose;

                objectDetectionMsg.name=recObj.name.get();
                objectDetectionMsg.type=recObj.type.get();
                objectDetectionMsg.width=recObj.width.get();
                objectDetectionMsg.height=recObj.height.get();
                objectDetectionMsg.depth=recObj.depth.get();

                chatter_pub.publish(objectDetectionMsg);

            }
        }
        currentid = 0;
    }
    return UIMA_ERR_NONE;
  }

  void publishDebugMarker(percepteros::EnclosingCircle circle){
      rs::StampedPose pose = circle.pose.get();
      std::vector<double> translation = pose.translation.get();
      visualization_msgs::Marker marker;
      marker.header.frame_id = pose.frame.get();
      marker.header.stamp = ros::Time();
      marker.ns = "my_namespace";
      marker.id = currentid;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = translation[0];
      marker.pose.position.y = translation[1];
      marker.pose.position.z = translation[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = circle.radius.get();
      marker.scale.y = circle.radius.get();
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      debug_pub.publish( marker );
      cout << "radius"<< circle.radius.get()<< endl;
      cout << "x" << translation[0] << endl;
      cout << "y" << translation[1] << endl;
      cout << "z" << translation[2] << endl;
      currentid++;
  }

  void drawImageWithLock(cv::Mat &disp)
  {

  }
};

MAKE_AE(ROSPublisher)
