#ifndef __CYLINDER_ANNOTATOR_H__
#define __CYLINDER_ANNOTATOR_H__

#include <vector>
#include <map>
#include <string>
#include <mutex>

#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/PointIndices.h>
#include <geometry_msgs/PoseStamped.h>


#include <pcl/visualization/pcl_visualizer.h>

#include <percepteros/types/all_types.h>

#include <uima/api.hpp>
using namespace uima;

class CylinderAnnotator : public DrawingAnnotator
{

typedef pcl::PointXYZRGBA PointT;

private:

    pcl::PointCloud<PointT>::Ptr cloud_ptr;
    std::vector<pcl::PointIndices> clusterIndices;
    double pointSize = 1;
    //default
    /*
    constexpr static double CYLINDER_NORMAL_WEIGHT = 0.024;
    constexpr static double CYLINDER_MIN_RADIUS = 0.01;
    constexpr static double CYLINDER_MAX_RADIUS = 0.05;
    constexpr static double CYLINDER_DISTANCE_THRESHOLD = 0.015;
    constexpr static double CYLINDER_MIN_RATIO = 0.5;
    constexpr static double CYLINDER_PLANE_DISTANCE_THRESHOLD = 0.01;
    constexpr static double CYLINDER_OVEREXTENSION_THRESHOLD = 0.003;
    constexpr static double CYLINDER_MAX_OVEREXTENSION_RATIO = 1250;
    constexpr static double NORMAL_SEARCH_RADIUS = 0.015;
    constexpr static int MAX_SEGMENTATION_ITERATIONS = 2000;
    constexpr static double EPSILON_ANGLE = 0.3;
    constexpr static int MIN_CLOUD_SIZE = 50;
    */

    constexpr static double CYLINDER_NORMAL_WEIGHT = 0.024;
    constexpr static double CYLINDER_MIN_RADIUS = 0.008;
    constexpr static double CYLINDER_MAX_RADIUS = 0.04;
    constexpr static double CYLINDER_DISTANCE_THRESHOLD = 0.020;
    constexpr static double CYLINDER_MIN_RATIO = 0.5;
    constexpr static double CYLINDER_PLANE_DISTANCE_THRESHOLD = 0.01;
    constexpr static double CYLINDER_OVEREXTENSION_THRESHOLD = 0.003;
    constexpr static double CYLINDER_MAX_OVEREXTENSION_RATIO = 1250;
    constexpr static double NORMAL_SEARCH_RADIUS = 0.015;
    constexpr static int MAX_SEGMENTATION_ITERATIONS = 2000;
    constexpr static double EPSILON_ANGLE = 0.3;
    constexpr static int MIN_CLOUD_SIZE = 50;
    /*
    constexpr static double CYLINDER_NORMAL_WEIGHT = 0.024;
    constexpr static double CYLINDER_MIN_RADIUS = 0.005;
    constexpr static double CYLINDER_MAX_RADIUS = 0.1;
    constexpr static double CYLINDER_DISTANCE_THRESHOLD = 0.1;
    constexpr static double CYLINDER_MIN_RATIO = 0.5;
    constexpr static double CYLINDER_PLANE_DISTANCE_THRESHOLD = 0.01;
    constexpr static double CYLINDER_OVEREXTENSION_THRESHOLD = 0.003;
    constexpr static double CYLINDER_MAX_OVEREXTENSION_RATIO = 1250;*/


public:

  CylinderAnnotator(): DrawingAnnotator(__func__){

  }

  TyErrorId initialize(uima::AnnotatorContext &ctx);

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec);

  TyErrorId destroy();

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);

  int segmentPlane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_input,
                   int model_type,
                   double distance,
                   pcl::ModelCoefficients::Ptr coefficients,
                   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_remaining,
                   Eigen::Vector3f axis = Eigen::Vector3f(1,0,0),
                   double epsilon = 0.1);


  Eigen::Vector3f vectorFromCoeff(pcl::ModelCoefficients::Ptr coefficients, int begin_index);

  int segmentCylinder(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_input,
                                       double normal_weight,
                                       double radius_min,
                                       double radius_max,
                                       double distance,
                                       pcl::ModelCoefficients::Ptr coefficients,
                                       pcl::PointIndices::Ptr cluster_indices);


  int isCylinder(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_object,
                 geometry_msgs::PoseStamped &pose, percepteros::RecognitionObject &o, CAS &tcas,
                 tf::Transform& transform);

  void detectObjectsOnTable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, CAS &tcas);


  float dot(pcl::PointXYZRGBNormal p1, Eigen::Vector3f p2)
  {
    return p1.x * p2.x() + p1.y * p2.y() + p1.z * p2.z();
  }

  float dot(pcl::PointXYZRGB p1, Eigen::Vector3f p2)
  {
    return p1.x * p2.x() + p1.y * p2.y() + p1.z * p2.z();
  }
};

#endif //__CYLINDER_ANNOTATOR_H__
