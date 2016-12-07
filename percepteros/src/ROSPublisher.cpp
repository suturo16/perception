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
        c.annotations.filter(objects);

        if(objects.size()!=0){
            for(percepteros::RecognitionObject recObj : objects){
                outInfo(recObj.name.get());

                suturo_perception_msgs::ObjectDetection objectDetectionMsg;

                tf::Stamped<tf::Pose> tf_stamped_pose;
                rs::conversion::from(recObj.pose.get(), tf_stamped_pose);

                objectDetectionMsg.pose.pose.position.x=tf_stamped_pose.getOrigin().getX();
                objectDetectionMsg.pose.pose.position.y=tf_stamped_pose.getOrigin().getY();
                objectDetectionMsg.pose.pose.position.z=tf_stamped_pose.getOrigin().getZ();
                objectDetectionMsg.pose.pose.orientation.x=tf_stamped_pose.getRotation().getX();
                objectDetectionMsg.pose.pose.orientation.y=tf_stamped_pose.getRotation().getY();
                objectDetectionMsg.pose.pose.orientation.z=tf_stamped_pose.getRotation().getZ();
                objectDetectionMsg.pose.pose.orientation.w=tf_stamped_pose.getRotation().getW();
                objectDetectionMsg.name=recObj.name.get();
                objectDetectionMsg.type=recObj.type.get();
                objectDetectionMsg.width=recObj.width.get();
                objectDetectionMsg.height=recObj.height.get();
                objectDetectionMsg.depth=recObj.depth.get();

                chatter_pub.publish(objectDetectionMsg);
            }
        }
    }

    ros::spinOnce();

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {

  }
};

MAKE_AE(ROSPublisher)
