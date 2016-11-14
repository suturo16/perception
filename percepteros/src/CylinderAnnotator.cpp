#include <../include/percepteROS/CylinderAnnotator.h>

  TyErrorId CylinderAnnotator::initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    std::string param;
    ctx.extractValue("test_param", param);
    return UIMA_ERR_NONE;
  }

  TyErrorId CylinderAnnotator::destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId CylinderAnnotator::process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD,*cloud_ptr);

    outInfo("Cloud size: " << cloud_ptr->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  int CylinderAnnotator::segmentPlane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_input,
                               int model_type,
                               double distance,
                               pcl::ModelCoefficients::Ptr coefficients,
                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_remaining,
                               Eigen::Vector3f axis,
                               double epsilon)
  {
    pcl::SACSegmentation <pcl::PointXYZRGBNormal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
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

  Eigen::Vector3f CylinderAnnotator::vectorFromCoeff(pcl::ModelCoefficients::Ptr coefficients, int begin_index)
  {
    return Eigen::Vector3f(coefficients->values[begin_index+0],
                           coefficients->values[begin_index+1],
                           coefficients->values[begin_index+2]);
  }

  int CylinderAnnotator::segmentCylinder(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_input,
                                       double normal_weight,
                                       double radius_min,
                                       double radius_max,
                                       double distance,
                                       pcl::ModelCoefficients::Ptr coefficients,
                                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_remaining)
  {
    pcl::SACSegmentationFromNormals <pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices <pcl::PointXYZRGBNormal> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight(normal_weight);
    seg.setRadiusLimits(radius_min, radius_max);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(MAX_SEGMENTATION_ITERATIONS);
    seg.setDistanceThreshold(distance);
    seg.setInputCloud(cloud_input);
    seg.setInputNormals(cloud_input);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud_input);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_remaining);

    return (int)inliers->indices.size();
  }


int CylinderAnnotator::isCylinder(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_object,
                                geometry_msgs::PoseStamped &pose) {

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rem1(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rem2(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);

  int cylinder_size = segmentCylinder(cloud_object, CYLINDER_NORMAL_WEIGHT, CYLINDER_MIN_RADIUS, CYLINDER_MAX_RADIUS,
                                      CYLINDER_DISTANCE_THRESHOLD,coefficients_cylinder, cloud_rem1);

  Eigen::Vector3f cylinder_axis_direction = vectorFromCoeff(coefficients_cylinder, 3);

  if ((double) cylinder_size / (double) cloud_object->width < CYLINDER_MIN_RATIO)
  {
    return 0;
  }

  segmentPlane(cloud_rem1, pcl::SACMODEL_PERPENDICULAR_PLANE, CYLINDER_PLANE_DISTANCE_THRESHOLD,
               coefficients_plane, cloud_rem2, cylinder_axis_direction, EPSILON_ANGLE);


  Eigen::Vector3f v1, v2, v3;

  float max_v3, min_v3;

  v3 = cylinder_axis_direction.normalized();
  v1 = v3.cross(Eigen::Vector3f::UnitX()).normalized();
  v2 = v3.cross(v1).normalized();

  max_v3 = min_v3 = dot(cloud_object->points.at(0), v3);

  Eigen::Vector3f point_on_axis = vectorFromCoeff(coefficients_cylinder, 0);

  float axis_v1 = v1.dot(point_on_axis);

  float axis_v2 = v2.dot(point_on_axis);

  float overextension = 0.0f;

  for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = cloud_object->points.begin();
      it < cloud_object->points.end(); it++)
  {
      float value_v1, value_v2, value_v3;

      value_v1 = dot(*it, v1);
      value_v2 = dot(*it, v2);
      value_v3 = dot(*it, v3);

      max_v3 = fmaxf(max_v3, value_v3);
      min_v3 = fminf(min_v3, value_v3);

      float d = (Eigen::Vector3f(value_v1, value_v2, 0.0f) - Eigen::Vector3f(axis_v1, axis_v2, 0.0f)).norm();
      if(d > coefficients_cylinder->values[6] + CYLINDER_OVEREXTENSION_THRESHOLD) {
        overextension += d - coefficients_cylinder->values[6] - CYLINDER_OVEREXTENSION_THRESHOLD;
      }
    }

    if(overextension * CYLINDER_MAX_OVEREXTENSION_RATIO > (float)cloud_object->points.size())
    {
      return 0;
    }

    Eigen::Matrix3f mat;
    mat << v1, v2, v3;
    Eigen::Quaternionf qua(mat);
    qua.normalize();

    pose.header.frame_id = cloud_object->header.frame_id;
    pose.pose.orientation.x = qua.x();
    pose.pose.orientation.y = qua.y();
    pose.pose.orientation.z = qua.z();
    pose.pose.orientation.w = qua.w();

    pose.pose.position.x = (max_v3 + min_v3) * v3.x() / 2.0f + axis_v2 * v2.x() + axis_v1 * v1.x();
    pose.pose.position.y = (max_v3 + min_v3) * v3.y() / 2.0f + axis_v2 * v2.y() + axis_v1 * v1.y();
    pose.pose.position.z = (max_v3 + min_v3) * v3.z() / 2.0f + axis_v2 * v2.z() + axis_v1 * v1.z();

    //TODO IMPORTANT: implement shape
    /*shape.height = max_v3 - min_v3;
    shape.width = shape.depth = coefficients_cylinder->values[6] * 2;
    shape.type = perception_msgs::Shape::CYLINDER;*/

    return (int)(cloud_object->points.size() - cloud_rem2->points.size());
}

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(CylinderAnnotator)
