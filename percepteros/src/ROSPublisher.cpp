// UIMA
#include <uima/api.hpp>

#include <opencv2/opencv.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <percepteros/types/all_types.h>
#include <suturo_perception_msgs/ObjectDetection.h>

using namespace uima;

class ROSPublisher : public DrawingAnnotator
{
private:

  ros::NodeHandle n;
  ros::Publisher chatter_pub;

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

     for(rs::Cluster c: clusters){
        std::vector<percepteros::RecognitionObject> objects;
        std::vector<rs::PoseAnnotation> poses;
        
        c.annotations.filter(objects);
        c.annotations.filter(poses);
        
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
            	Eigen::Quaterniond q(mat);
            	
                outInfo(recObj.name.get());

                suturo_perception_msgs::ObjectDetection objectDetectionMsg;
                
                objectDetectionMsg.pose.header.frame_id = "head_mount_kinect_rgb_optical_frame";

                objectDetectionMsg.pose.pose.position.x=translation[0];
                objectDetectionMsg.pose.pose.position.y=translation[1];
                objectDetectionMsg.pose.pose.position.z=translation[2];
                
                objectDetectionMsg.pose.pose.orientation.x=q.x();
                objectDetectionMsg.pose.pose.orientation.y=q.y();
                objectDetectionMsg.pose.pose.orientation.z=q.z();
                objectDetectionMsg.pose.pose.orientation.w=q.w();
                objectDetectionMsg.name=recObj.name.get();
                objectDetectionMsg.type=recObj.type.get();
                objectDetectionMsg.width=recObj.width.get();
                objectDetectionMsg.height=recObj.height.get();
                objectDetectionMsg.depth=recObj.depth.get();

                chatter_pub.publish(objectDetectionMsg);
            }
        }
    }
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {

  }
};

MAKE_AE(ROSPublisher)
