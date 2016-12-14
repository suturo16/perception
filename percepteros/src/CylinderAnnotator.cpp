#include <../include/percepteros/CylinderAnnotator.h>
#include <rs/types/all_types.h>

typedef pcl::PointXYZRGBA PointT;

  TyErrorId CylinderAnnotator::initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
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
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normal_ptr);

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
      // Create the filtering object
      /*pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
      sor.setInputCloud (cloud_cluster_normal);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*cloud_cluster_normal);*/

      geometry_msgs::PoseStamped pose;
      percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);

      int cyl = isCylinder(cloud_cluster_normal, pose, o, tcas);
      if(cyl){
          outInfo("Pose:x:" << pose.pose.position.x << " y:" << pose.pose.position.y << " z:" << pose.pose.position.z);
          outInfo("took: " << clock.getTime() << " ms.");
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
      }

    }
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
                                geometry_msgs::PoseStamped &pose, percepteros::RecognitionObject &o, CAS &tcas) {

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rem1(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rem2(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);

  int cylinder_size = segmentCylinder(cloud_object, CYLINDER_NORMAL_WEIGHT, CYLINDER_MIN_RADIUS, CYLINDER_MAX_RADIUS,
                                      CYLINDER_DISTANCE_THRESHOLD,coefficients_cylinder, cloud_rem1);

  if(coefficients_cylinder->values.size()==0){
      outInfo("No Cylinder found.");
      return 0;
  }

  Eigen::Vector3f cylinder_axis_direction = vectorFromCoeff(coefficients_cylinder, 3);

  if ((double) cylinder_size / (double) cloud_object->width < CYLINDER_MIN_RATIO)
  {
    outInfo("No Cylinder found.");
    return 0;
  }
  outInfo("Found Cylinder.");
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

    float height = max_v3 - min_v3;
    float width= coefficients_cylinder->values[6] * 2;
    float depth = width;

    if(height<0.01||width<0.01||depth<0.01){
        return 0;
    }

    tf::Vector3 trans(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    tf::Matrix3x3 rot;
    rot.setValue(mat(0,0), mat(0,1), mat(0,2),
                 mat(1,0), mat(1,1), mat(1,2),
                 mat(2,0), mat(2,1), mat(2,2));

    transform.setOrigin(trans);
    transform.setBasis(rot);

    o.name.set("Cylinder");
    o.type.set(2);
    o.width.set(width);
    o.height.set(height);
    o.depth.set(depth);

    return (int)(cloud_object->points.size() - cloud_rem2->points.size());
}
// This macro exports an entry point that is used to create the annotator.
MAKE_AE(CylinderAnnotator)
