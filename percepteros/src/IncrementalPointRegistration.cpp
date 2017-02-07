#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace uima;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;



class IncrementalPointRegistration : public DrawingAnnotator
{
private:
  float test_param;

  //its left and right viewports
  int vp_1, vp_2;

  PointCloud::Ptr lastResult;
  double pointSize = 1;

public:

  IncrementalPointRegistration(): DrawingAnnotator(__func__){
      lastResult = PointCloud::Ptr(new PointCloud);
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
    PointCloud::Ptr cloud_ptr(new PointCloud);
    outInfo("Cloud size =  " << lastResult->size());
    cas.get(VIEW_CLOUD,*cloud_ptr);
    if(lastResult->points.size() == 0){
        lastResult = cloud_ptr;
        return UIMA_ERR_NONE;
    }

    PointCloud::Ptr result (new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

    source = cloud_ptr;
    target = lastResult;

    // Add visualization data
    //showCloudsLeft(source, target);

    PointCloud::Ptr temp (new PointCloud);
    //PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);
    lastResult->operator +=(*result);

    outInfo("Height: " << lastResult->height);

    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

    PointCloud::Ptr temp2 (new PointCloud);

    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (lastResult);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*temp2);

    lastResult = temp2;

    return UIMA_ERR_NONE;
  }

  struct PCD
  {
    PointCloud::Ptr cloud;
    std::string f_name;

    PCD() : cloud (new PointCloud) {};
  };

  struct PCDComparator
  {
    bool operator () (const PCD& p1, const PCD& p2)
    {
      return (p1.f_name < p2.f_name);
    }
  };


  // Define a new point representation for < x, y, z, curvature >
  class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
  {
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
  public:
    MyPointRepresentation ()
    {
      // Define the number of dimensions
      nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
      // < x, y, z, curvature >
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
      out[3] = p.curvature;
    }
  };




  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Align a pair of PointCloud datasets and return the result
    * \param cloud_src the source PointCloud
    * \param cloud_tgt the target PointCloud
    * \param output the resultant aligned source PointCloud
    * \param final_transform the resultant transform between source and target
    */
  void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
  {
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
      grid.setLeafSize (0.05, 0.05, 0.05);
      grid.setInputCloud (cloud_src);
      grid.filter (*src);

      grid.setInputCloud (cloud_tgt);
      grid.filter (*tgt);
    }
    else
    {
      src = cloud_src;
      tgt = cloud_tgt;
    }


    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);
    // Set the point representation
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);



    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);
    for (int i = 0; i < 30; ++i)
    {
      PCL_INFO ("Iteration Nr. %d.\n", i);

      // save cloud for visualization purpose
      points_with_normals_src = reg_result;

      // Estimate
      reg.setInputSource (points_with_normals_src);
      reg.align (*reg_result);

          //accumulate transformation between each Iteration
      Ti = reg.getFinalTransformation () * Ti;

          //if the difference between this transformation and the previous one
          //is smaller than the threshold, refine the process by reducing
          //the maximal correspondence distance
      if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
        reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

      prev = reg.getLastIncrementalTransformation ();
    }

      //
    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    //
    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
   }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;

    if(firstRun)
    {
      visualizer.addPointCloud(lastResult, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(lastResult, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(IncrementalPointRegistration)
