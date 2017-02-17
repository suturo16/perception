#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <percepteros/types/all_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/visualization/pcl_visualizer.h>


using namespace uima;

typedef pcl::PointXYZRGBA PointT;

class CakeAnnotator : public DrawingAnnotator
{
private:
  float test_param;
  tf::Transform transform;

  pcl::PointCloud<PointT>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rem1;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rem2;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rem3;

  std::vector<pcl::PointIndices> clusterIndices;
  std::vector<pcl::PointIndices> clusterIndices1;
  std::vector<pcl::PointIndices> clusterIndices2;
  std::vector<pcl::PointIndices> clusterIndices3;

  double pointSize = 1;

  constexpr static double BOX_NORMAL_WEIGHT = 0.04;
  constexpr static double BOX_DISTANCE_THRESHOLD_NORMALPLANE = 0.015;
  constexpr static double BOX_DISTANCE_THRESHOLD_PLANE1 = 0.004;
  constexpr static double BOX_DISTANCE_THRESHOLD_PLANE2 = 0.01;
  constexpr static double BOX_DISTANCE_THRESHOLD_PLANE3 = 0.01;
  constexpr static double BOX_EPSILON_PLANE1 = 0.1;
  constexpr static double BOX_MAX_SIZE_RATIO_PLANE1 = 0.75;
  constexpr static double BOX_MIN_SIZE_RATIO_PLANE1 = 0.2;
  constexpr static double BOX_MIN_SIZE_RATIO_PLANE2 = 0.25;
  constexpr static double BOX_MIN_SIZE_RATIO_PLANE3 = 0.35;
  //constexpr static double PLANE2_TO_BOX_MIN_RATIO = 0.20;
  constexpr static double BOX_MIN_MATCHED_POINTS_RATIO = 0.3;
  constexpr static int MAX_SEGMENTATION_ITERATIONS = 2000;
  constexpr static double EPSILON_ANGLE = 0.3;
  constexpr static int MIN_CLOUD_SIZE = 50;


public:

  CakeAnnotator(): DrawingAnnotator(__func__){

      cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", test_param);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);

    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normal_ptr);

    clusterIndices.clear();
    clusterIndices1.clear();
    clusterIndices2.clear();
    clusterIndices3.clear();


    for(auto cluster : clusters)
    {
      pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
      rs::ReferenceClusterPoints clusterpoints(cluster.points());
      rs::conversion::from(clusterpoints.indices(), *cluster_indices);

      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<pcl::Normal>::Ptr cluster_normal(new pcl::PointCloud<pcl::Normal>());

      for(std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
          pit != cluster_indices->indices.end(); pit++)
      {
        cluster_cloud->points.push_back(cloud_ptr->points[*pit]);
        cluster_normal->points.push_back(normal_ptr->points[*pit]);
      }
      cluster_cloud->width = cluster_cloud->points.size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;
      cluster_normal->width = cluster_normal->points.size();
      cluster_normal->height = 1;
      cluster_normal->is_dense = true;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusterRGB (new pcl::PointCloud<pcl::PointXYZRGB>);

      pcl::copyPointCloud(*cluster_cloud,*cloud_clusterRGB);
      pcl::concatenateFields (*cloud_clusterRGB, *cluster_normal, *cloud_cluster_normal);

       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

       // Create the filtering object
       /*pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
       sor.setInputCloud (cloud_cluster_normal);
       sor.setLeafSize (0.01f, 0.01f, 0.01f);
       sor.filter (*cloud_filtered);*/

      geometry_msgs::PoseStamped pose;
      percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
      tf::Transform transform;
      int box = isBox(cloud_cluster_normal, pose, o, transform, cluster_indices);
      if(box){
          clusterIndices.push_back(*cluster_indices);
          outInfo("Box");
          cluster.annotations.append(o);

          tf::StampedTransform camToWorld;
          camToWorld.setIdentity();
          if(scene.viewPoint.has())
          {
            rs::conversion::from(scene.viewPoint.get(), camToWorld);
          }

          tf::Stamped<tf::Pose> camera(transform, camToWorld.stamp_, camToWorld.child_frame_id_);
          tf::Stamped<tf::Pose> world(camToWorld*transform, camToWorld.stamp_, camToWorld.frame_id_);

          rs::PoseAnnotation poseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
          poseAnnotation.camera.set(rs::conversion::to(tcas, camera));
          poseAnnotation.world.set(rs::conversion::to(tcas, world));
          poseAnnotation.source.set("3DEstimate");
          cluster.annotations.append(poseAnnotation);
          scene.identifiables.append(cluster);
      } else{
          outInfo("No Box");
      }

    }
    return UIMA_ERR_NONE;
  }

  int segmentPlaneFromNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_input,
                                          int model_type,
                                          double normal_weight,
                                          double distance,
                                          pcl::ModelCoefficients::Ptr coefficients,
                                          pcl::PointIndices::Ptr inliers,
                                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_remaining,
                                          Eigen::Vector3f axis = Eigen::Vector3f(1,0,0),
                                          double epsilon = 0.1)
  {
    pcl::SACSegmentationFromNormals <pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> seg;
    pcl::ExtractIndices <pcl::PointXYZRGBNormal> extract;

    // Find largest planar component with RANSAC
    seg.setOptimizeCoefficients(true);
    seg.setModelType(model_type);
    seg.setNormalDistanceWeight(normal_weight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(MAX_SEGMENTATION_ITERATIONS);
    seg.setDistanceThreshold(distance);
    seg.setAxis(axis);
    seg.setEpsAngle(epsilon);
    seg.setInputCloud(cloud_input);
    seg.setInputNormals(cloud_input);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud_input);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_remaining);

    return (int)inliers->indices.size();
  }

  float dot(pcl::PointXYZRGBNormal p1, Eigen::Vector3f p2)
  {
    return p1.x * p2.x() + p1.y * p2.y() + p1.z * p2.z();
  }

  float dot(pcl::PointXYZRGB p1, Eigen::Vector3f p2)
  {
    return p1.x * p2.x() + p1.y * p2.y() + p1.z * p2.z();
  }

  int segmentPlane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_input,
                               int model_type,
                               double distance,
                               pcl::ModelCoefficients::Ptr coefficients,
                               pcl::PointIndices::Ptr inliers,
                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_remaining,
                               Eigen::Vector3f axis = Eigen::Vector3f(1,0,0),
                               double epsilon = 0.1)
  {
    pcl::SACSegmentation <pcl::PointXYZRGBNormal> seg;
    pcl::ExtractIndices <pcl::PointXYZRGBNormal> extract;

    seg.setInputCloud(cloud_input);
    seg.setModelType(model_type);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(MAX_SEGMENTATION_ITERATIONS);
    seg.setDistanceThreshold(distance);
    seg.setAxis(axis);
    seg.setEpsAngle(epsilon);
    seg.setOptimizeCoefficients(true);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud_input);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_remaining);

    return (int)inliers->indices.size();
  }

  Eigen::Vector3f vectorFromCoeff(pcl::ModelCoefficients::Ptr coefficients, int begin_index)
  {
    return Eigen::Vector3f(coefficients->values[begin_index+0],
                           coefficients->values[begin_index+1],
                           coefficients->values[begin_index+2]);
  }


  int isBox(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_object,
                             geometry_msgs::PoseStamped &pose,
                             percepteros::RecognitionObject& o,
                             tf::Transform& transform,
                             pcl::PointIndices::Ptr cluster_indices)
  {
        cloud_rem1 = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        cloud_rem2 = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        cloud_rem3 = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        pcl::ModelCoefficients::Ptr coefficients_plane1(new pcl::ModelCoefficients ());
        pcl::ModelCoefficients::Ptr coefficients_plane2(new pcl::ModelCoefficients ());
        pcl::ModelCoefficients::Ptr coefficients_plane3(new pcl::ModelCoefficients ());

        pcl::PointIndices::Ptr inliers1(new pcl::PointIndices());
        pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
        pcl::PointIndices::Ptr inliers3(new pcl::PointIndices());


        int matched_points = segmentPlaneFromNormals(cloud_object, pcl::SACMODEL_NORMAL_PLANE, BOX_NORMAL_WEIGHT,
                                                     BOX_DISTANCE_THRESHOLD_NORMALPLANE, coefficients_plane1, inliers1, cloud_rem1);
        clusterIndices1.push_back(*inliers1);

        Eigen::Vector3f norm_plane1 = vectorFromCoeff(coefficients_plane1,0);

        pcl::ModelCoefficients::Ptr co(new pcl::ModelCoefficients ());
        int plane_size = segmentPlane(cloud_object, pcl::SACMODEL_PERPENDICULAR_PLANE, BOX_DISTANCE_THRESHOLD_PLANE1,
                                      co, inliers1, cloud_rem1,norm_plane1, BOX_EPSILON_PLANE1);

        // return false if the plane is too small
        if ((double) matched_points / (double) cloud_object->points.size() < BOX_MIN_SIZE_RATIO_PLANE1)
        {
          return 0;
        }

        // return false if the complete Object is a single plane
        if ((double) plane_size / (double) cloud_object->points.size() > BOX_MAX_SIZE_RATIO_PLANE1)
        {
          return 0;
        }

        plane_size = segmentPlane(cloud_rem1, pcl::SACMODEL_PARALLEL_PLANE, BOX_DISTANCE_THRESHOLD_PLANE2,
                                   coefficients_plane2, inliers2, cloud_rem2, norm_plane1, EPSILON_ANGLE);
        clusterIndices2.push_back(*inliers2);


        matched_points += plane_size;

        Eigen::Vector3f norm_plane2 = vectorFromCoeff(coefficients_plane2,0);

        /*if((double) plane_size / (double) cloud_object->points.size() < PLANE2_TO_BOX_MIN_RATIO) {
            return 0;
        }*/

        if ((double) plane_size / (double) cloud_rem1->points.size() < BOX_MIN_SIZE_RATIO_PLANE2)
        {
          return 0;
        }

        if ((double) matched_points / (double) cloud_object->points.size() < BOX_MIN_MATCHED_POINTS_RATIO)
        {

          if(cloud_rem2->points.size() < MIN_CLOUD_SIZE)
          {
            return 0;
          }

          plane_size = segmentPlane(cloud_rem2, pcl::SACMODEL_PARALLEL_PLANE,
                                    BOX_DISTANCE_THRESHOLD_PLANE3, coefficients_plane3, inliers3, cloud_rem3, norm_plane1, EPSILON_ANGLE);
          clusterIndices3.push_back(*inliers3);

          matched_points += plane_size;

          Eigen::Vector3f norm_plane3 = vectorFromCoeff(coefficients_plane3,0);


          if ((double) plane_size / (double) cloud_rem2->points.size() < BOX_MIN_SIZE_RATIO_PLANE3)
          {
            return 0;
          }

          double angle = acos(norm_plane2.dot(norm_plane3));

          if (angle > M_PI_2 + EPSILON_ANGLE || angle < M_PI_2 - EPSILON_ANGLE)
          {
            return 0;
          }
        }

        Eigen::Vector3f v1, v2, v3;

        float max_v1, max_v2, max_v3, min_v1, min_v2, min_v3;


        v1 = norm_plane1.normalized();
        v2 = v1.cross(norm_plane2).normalized();
        v3 = v1.cross(v2).normalized();

        max_v1 = min_v1 = dot(cloud_object->points.at(0), v1);
        max_v2 = min_v2 = dot(cloud_object->points.at(0), v2);
        max_v3 = min_v3 = dot(cloud_object->points.at(0), v3);

        for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = cloud_object->points.begin();
            it < cloud_object->points.end(); it++)
        {
          float value_v1, value_v2, value_v3;

          value_v1 = dot(*it, v1);
          value_v2 = dot(*it, v2);
          value_v3 = dot(*it, v3);

          max_v1 = fmaxf(max_v1, value_v1);
          max_v2 = fmaxf(max_v2, value_v2);
          max_v3 = fmaxf(max_v3, value_v3);
          min_v1 = fminf(min_v1, value_v1);
          min_v2 = fminf(min_v2, value_v2);
          min_v3 = fminf(min_v3, value_v3);
        }

        /*min_v1 = -coefficients_plane1->values[3];
        min_v2 += 0.005;
        min_v2 += 0.005;
        */

        Eigen::Matrix3f mat;
        mat << v1, v2, v3;
        Eigen::Quaternionf qua(mat);
        qua.normalize();

        pose.header.frame_id = cloud_object->header.frame_id;
        pose.pose.orientation.x = qua.x();
        pose.pose.orientation.y = qua.y();
        pose.pose.orientation.z = qua.z();
        pose.pose.orientation.w = qua.w();

        pose.pose.position.x =
            ((max_v1 + min_v1) * v1.x() + (max_v2 + min_v2) * v2.x() + (max_v3 + min_v3) * v3.x()) / 2.0f;
        pose.pose.position.y =
            ((max_v1 + min_v1) * v1.y() + (max_v2 + min_v2) * v2.y() + (max_v3 + min_v3) * v3.y()) / 2.0f;
        pose.pose.position.z =
            ((max_v1 + min_v1) * v1.z() + (max_v2 + min_v2) * v2.z() + (max_v3 + min_v3) * v3.z()) / 2.0f;


        tf::Vector3 trans(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        tf::Matrix3x3 rot;

        rot.setValue(mat(0,0), mat(0,1), mat(0,2),
                     mat(1,0), mat(1,1), mat(1,2),
                     mat(2,0), mat(2,1), mat(2,2));

        transform.setOrigin(trans);
        transform.setBasis(rot);

        o.name.set("Box");
        o.type.set(1);
        o.width.set(max_v1 - min_v1);
        o.height.set(max_v3 - min_v3);
        o.depth.set(max_v2 - min_v2);

        return matched_points;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;
    for(size_t i = 0; i < clusterIndices.size(); ++i)
    {
      const pcl::PointIndices &indices = clusterIndices[i];
      for(size_t j = 0; j < indices.indices.size(); ++j)
      {
        size_t index = indices.indices[j];
        cloud_ptr->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
      }
    }
    for(size_t i = 0; i < clusterIndices1.size(); ++i)
    {
      outInfo("clusters:" << clusterIndices1.size());
      const pcl::PointIndices &indices = clusterIndices1[i];
      for(size_t j = 0; j < indices.indices.size(); ++j)
      {
        size_t index = indices.indices[j];
        cloud_ptr->points[index].rgba = rs::common::colors[1];
      }
    }

    for(size_t i = 0; i < clusterIndices2.size(); ++i)
    {
      const pcl::PointIndices &indices = clusterIndices2[i];
      for(size_t j = 0; j < indices.indices.size(); ++j)
      {
        size_t index = indices.indices[j];
        cloud_ptr->points[index].rgba = rs::common::colors[2];
      }
    }

    for(size_t i = 0; i < clusterIndices3.size(); ++i)
    {
      const pcl::PointIndices &indices = clusterIndices3[i];
      for(size_t j = 0; j < indices.indices.size(); ++j)
      {
        size_t index = indices.indices[j];
        cloud_ptr->points[index].rgba = rs::common::colors[3];
      }
    }
    if(firstRun)
    {
      visualizer.addPointCloud(cloud_ptr, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud_ptr, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(CakeAnnotator)
