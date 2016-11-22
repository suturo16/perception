#include </home/surxz/caterros/src/perception/percepteros/include/percepteROS/CylinderAnnotator.h>

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
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudNormal_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);

    cas.get(VIEW_CLOUD,*cloudA_ptr);
    cas.get(VIEW_NORMALS, *normal_ptr);
    pcl::copyPointCloud(*cloudA_ptr,*cloud_ptr);
    pcl::concatenateFields (*cloud_ptr, *normal_ptr, *cloudNormal_ptr);
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud (cloudNormal_ptr);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloudNormal_ptr);

    geometry_msgs::PoseStamped pose;

    detectObjectsOnTable(cloud_ptr);


    outInfo("Pose:x:" << pose.pose.position.x << " y:" << pose.pose.position.y << " z:" << pose.pose.position.z);
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

    //TODO IMPORTANT: implement shape
    /*shape.height = max_v3 - min_v3;
    shape.width = shape.depth = coefficients_cylinder->values[6] * 2;
    shape.type = perception_msgs::Shape::CYLINDER;*/

    return (int)(cloud_object->points.size() - cloud_rem2->points.size());
}

void CylinderAnnotator::detectObjectsOnTable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud (cloud_cluster);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*cloud_normals);

      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::concatenateFields(*cloud_cluster,*cloud_normals,*cloud_cluster_normal);
      geometry_msgs::PoseStamped pose;
      isCylinder(cloud_cluster_normal, pose);
    }
}
// This macro exports an entry point that is used to create the annotator.
MAKE_AE(CylinderAnnotator)
