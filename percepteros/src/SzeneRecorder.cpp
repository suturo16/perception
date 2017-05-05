#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <std_srvs/Empty.h>
#include <suturo_perception_msgs/PerceiveAction.h>
#include <ros/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

using namespace uima;


class SzeneRecorder : public Annotator
{
private:

    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointNormal PointNormalT;
    typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

    struct SavedSzene{

        cv::Mat beforeDepthImage;
        PointCloud::Ptr beforeAction;
        ros::Time beforeTime;

        cv::Mat afterDepthImage;
        PointCloud::Ptr afterAction;
        ros::Time afterTime;
    };

    ros::NodeHandle nh_;

    ros::ServiceServer beforeActionService, actionEndedService;

    bool queueSaveImage = false;

    bool firstRun = true;

    SavedSzene savedSzene;

public:

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


  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    nh_ = ros::NodeHandle("~");

    beforeActionService = nh_.advertiseService("before_action", &SzeneRecorder::beforeActionCallback, this);
    actionEndedService = nh_.advertiseService("perceive_action_effect", &SzeneRecorder::perceiveActionEffectCallback, this);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    cv::Mat depth_image;

    cas.get(VIEW_DEPTH_IMAGE, depth_image);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD,*cloud_ptr);

    outInfo("Cloud size: " << cloud_ptr->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    if(queueSaveImage){
        queueSaveImage = false;
        if(firstRun){
            firstRun = false;
            savedSzene.beforeAction = cloud_ptr;
            savedSzene.beforeDepthImage = depth_image;
        }
        else{
            savedSzene.afterAction = cloud_ptr;
            savedSzene.afterDepthImage = depth_image;
            startAlignment();
        }
    }
    return UIMA_ERR_NONE;
  }




  bool beforeActionCallback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& res){
      queueSaveImage = true;
      savedSzene.beforeTime = ros::Time::now();
      return true;
  }

  bool perceiveActionEffectCallback(suturo_perception_msgs::PerceiveAction::Request &req,
                                    suturo_perception_msgs::PerceiveAction::Response &res){
      queueSaveImage = true;
      savedSzene.afterTime = ros::Time::now();
      return true;
  }

  void startAlignment(){
    PointCloud::Ptr target;
    Eigen::Matrix4f pairTransform;

    pairAlign(savedSzene.afterAction, savedSzene.beforeAction, target, pairTransform, true);
  }

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

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SzeneRecorder)
