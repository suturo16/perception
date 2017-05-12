#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/output.h>

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <opencv2/highgui/highgui.hpp>

using namespace uima;


class ARMarkersDetector : public DrawingAnnotator
{

  typedef pcl::PointXYZRGBA  PointT;

public:
  cv::Mat image, cameraMatrix, distCoefficients;
  sensor_msgs::CameraInfo cam_info;
  pcl::PointCloud<PointT>::Ptr cloud_ptr;
  double pointSize;

  aruco::MarkerDetector qrDetector;
  aruco::CameraParameters camParams;
  float marker_size;
  std::vector<aruco::Marker> markers;
  std::vector<tf::Stamped<tf::Pose> > marker_poses_cam_frame;

  ARMarkersDetector() : DrawingAnnotator(__func__),  cloud_ptr(new pcl::PointCloud<PointT>()),
    pointSize(1),marker_size(0.0)
  {
    cameraMatrix = cv::Mat(3, 3, CV_64F);
    distCoefficients = cv::Mat(1, 8, CV_64F);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("marker_size"))
    {
      ctx.extractValue("marker_size", marker_size);
    }

    return UIMA_ERR_NONE;
  }
  TyErrorId typeSystemInit(TypeSystem const &type_system)
  {
    outInfo("typeSystemInit");
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
    outInfo("process begins");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    cas.get(VIEW_COLOR_IMAGE_HD,image);
    cas.get(VIEW_CAMERA_INFO_HD, cam_info);
    cas.get(VIEW_CLOUD,*cloud_ptr);

    readCameraInfo(cam_info);
    camParams.setParams(cameraMatrix, distCoefficients,cv::Size(cam_info.width,cam_info.height));

    outInfo(cameraMatrix);
    markers.clear();
    marker_poses_cam_frame.clear();
    qrDetector.detect(image, markers, camParams,marker_size);
    for(size_t i = 0;  i < markers.size(); ++i)
    {
      aruco::Marker &m = markers[i];
      rs::ARMarker marker_annotation =  rs::create<rs::ARMarker>(tcas);
      std::stringstream strstream;
      outInfo("Marker "<<m.id<< " found.");
      strstream<<m.id;
      marker_annotation.name.set(strstream.str());

      tf::Stamped<tf::Pose> pose;
      pose.setOrigin(tf::Vector3(m.Tvec.at<float>(0),m.Tvec.at<float>(1),m.Tvec.at<float>(2)));
      cv::Mat rot_mat(3,3,cv::DataType<float>::type);
      cv::Rodrigues(m.Rvec,rot_mat);
      tf::Quaternion rot;
      tf::Matrix3x3 tf_rot(rot_mat.at<float>(0,0), rot_mat.at<float>(0,1) ,rot_mat.at<float>(0,2),
          rot_mat.at<float>(1,0),rot_mat.at<float>(1,1),rot_mat.at<float>(1,2),
          rot_mat.at<float>(2,0),rot_mat.at<float>(2,1),rot_mat.at<float>(2,2));
      tf_rot.getRotation(rot);
      pose.setRotation(rot);
      pose.frame_id_ = cam_info.header.frame_id;
      marker_annotation.pose.set(rs::conversion::to(tcas,pose));
      scene.annotations.append(marker_annotation);
      marker_poses_cam_frame.push_back(pose);
    }
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void readCameraInfo(const sensor_msgs::CameraInfo &camInfo)
  {
    double *it = cameraMatrix.ptr<double>(0);
    *it++ = camInfo.K[0];
    *it++ = camInfo.K[1];
    *it++ = camInfo.K[2];
    *it++ = camInfo.K[3];
    *it++ = camInfo.K[4];
    *it++ = camInfo.K[5];
    *it++ = camInfo.K[6];
    *it++ = camInfo.K[7];
    *it++ = camInfo.K[8];

    distCoefficients = cv::Mat(1, camInfo.D.size(), CV_64F);
    it = distCoefficients.ptr<double>(0);
    for(size_t i = 0; i   < camInfo.D.size(); ++i, ++it)
    {
      *it = camInfo.D[i];
    }
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    disp = image.clone();
    for(size_t i = 0; i < markers.size(); ++i)
    {
      markers[i].draw(disp, cv::Scalar(0, 0, 255), 2);
      aruco::CvDrawingUtils::draw3dAxis(disp, markers[i], camParams);
      aruco::CvDrawingUtils::draw3dCube(disp, markers[i], camParams);
    }
  }
  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;

    if(firstRun)
    {
      visualizer.addPointCloud(cloud_ptr, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud_ptr, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
    }

    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0.2, 0, 0), 1, 0, 0, "X");
    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0.2, 0), 0, 1, 0, "Y");
    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 0.2), 0, 0, 1, "Z");

    for(size_t i = 0; i < marker_poses_cam_frame.size(); ++i)
    {
      std::ostringstream oss;
      oss << "marker_" << i;


      tf::Vector3 originB = marker_poses_cam_frame[i]* tf::Vector3(0, 0, 0);

      tf::Vector3 lineXB = marker_poses_cam_frame[i] * tf::Vector3(0.2, 0, 0);
      tf::Vector3 lineYB = marker_poses_cam_frame[i] * tf::Vector3(0, 0.2, 0);
      tf::Vector3 lineZB = marker_poses_cam_frame[i] * tf::Vector3(0, 0, 0.2);

      pcl::PointXYZ pclOriginB(originB.x(), originB.y(), originB.z());
      pcl::PointXYZ pclLineXB(lineXB.x(), lineXB.y(), lineXB.z());
      pcl::PointXYZ pclLineYB(lineYB.x(), lineYB.y(), lineYB.z());
      pcl::PointXYZ pclLineZB(lineZB.x(), lineZB.y(), lineZB.z());

      visualizer.addLine(pclOriginB, pclLineXB, 1, 0, 0, "lineX_" + oss.str());
      visualizer.addLine(pclOriginB, pclLineYB, 0, 1, 0, "lineY_" + oss.str());
      visualizer.addLine(pclOriginB, pclLineZB, 0, 0, 1, "lineZ_" + oss.str());

    }

  }
};

MAKE_AE(ARMarkersDetector)
