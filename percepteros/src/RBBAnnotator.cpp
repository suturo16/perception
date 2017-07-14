#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
#include <rs/DrawingAnnotator.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/transforms.h>
#include <opencv2/opencv.hpp>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <percepteros/Miniball.h>
#include <percepteros/types/all_types.h>


using namespace uima;
using namespace cv;
typedef pcl::PointXYZRGBA PointT;


class RBBAnnotator : public DrawingAnnotator
{
private:
  float test_param;

  tf::StampedTransform camToWorld, worldToCam;
  Eigen::Affine3d eigenTransform;
  Mat coords;
  double pointSize = 1;
  pcl::PointCloud<PointT>::Ptr cloud;
  tf::TransformListener listener;

public:

  RBBAnnotator():DrawingAnnotator(__func__){

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

  void cloud_to_img(
          const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
          cv::Mat &coords)
  {
      coords = cv::Mat(3, cloud->points.size(), CV_64FC1);
      for(int i=0;i<cloud->points.size();i++)
      {
          coords.at<double>(0,i) = cloud->points.at(i).x;
          coords.at<double>(1,i) = cloud->points.at(i).y;
          coords.at<double>(2,i) = cloud->points.at(i).z;
      }
  }

  void doMiniball(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, double& radius, double& x, double& y){
        int d = 2;            // dimension
        int n = cloud->points.size();      // number of points

        // generate random points and store them in a 2-d array
        // ----------------------------------------------------
        double** ap = new double*[n];
        for (int i=0; i<n; ++i) {
          double* p = new double[d];
          p[0] = cloud->points.at(i).x;
          p[1] = cloud->points.at(i).y;
          ap[i] = p;
        }

        // define the types of iterators through the points and their coordinates
        // ----------------------------------------------------------------------
        typedef double* const* PointIterator;
        typedef const double* CoordIterator;

        // create an instance of Miniball
        // ------------------------------
        typedef Miniball::
          Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> >
          MB;
        MB mb (d, ap, ap+n);

        // output results
        // --------------
        // center
        std::cout << "Center:\n  ";
        const double* center = mb.center();
        for(int i=0; i<d; ++i, ++center){
          std::cout << *center << " ";
          if(i==0){
              x=*center;
          } else{
              y=*center;
          }
        }
        std::cout << std::endl;

        // squared radius
        std::cout << "Squared radius:\n  ";
        std::cout << mb.squared_radius() <<  std::endl;

        // number of support points
        std::cout << "Number of support points:\n  ";
        std::cout << mb.nr_support_points() << std::endl;

        // support points on the boundary determine the smallest enclosing ball
        std::cout << "Support point indices (numbers refer to the input order):\n  ";
        MB::SupportPointIterator it = mb.support_points_begin();
        for (; it != mb.support_points_end(); ++it) {
          std::cout << (*it)-ap << " "; // 0 = first point
        }
        std::cout << std::endl;

        // relative error: by how much does the ball fail to contain all points?
        //                 tiny positive numbers come from roundoff and are ok
        std::cout << "Relative error:\n  ";
        double suboptimality;
        std::cout << mb.relative_error (suboptimality) <<  std::endl;

        // suboptimality: by how much does the ball fail to be the smallest
        //                enclosing ball of its support points? should be 0
        //                in most cases, but tiny positive numbers are again ok
        std::cout << "Suboptimality:\n  ";
        std::cout << suboptimality <<  std::endl;

        // validity: the ball is considered valid if the relative error is tiny
        //           (<= 10 times the machine epsilon) and the suboptimality is zero
        std::cout << "Validity:\n  ";
        std::cout << (mb.is_valid() ? "ok" : "possibly invalid") << std::endl;

        // computation time
        std::cout << "Computation time was "<< mb.get_time() << " seconds\n";

        radius = mb.squared_radius();
        // clean up
        for (int i=0; i<n; ++i)
          delete[] ap[i];
        delete[] ap;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
      const std::string &cloudname = this->name;
      if(cloud && cloud->size()>0){
          if(firstRun)
          {
            visualizer.addPointCloud(cloud, cloudname);
            visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
          }
          else
          {
            visualizer.updatePointCloud(cloud, cloudname);
            visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
          }
      }

  }

  void drawImageWithLock(cv::Mat &disp)
  {
    //disp=coords.clone();
  }



  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    outInfo("Test param =  " << test_param);
    cas.get(VIEW_CLOUD,*cloud_ptr);

    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    camToWorld.setIdentity();
    if(scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outInfo("No camera to world transformation!!!");
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }
    worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);

    tf::StampedTransform transform;
    listener.lookupTransform("/head_mount_kinect_rgb_optical_frame","/iai_kitchen/counter_top_island_link",ros::Time(0),transform);
    tf::transformTFToEigen(camToWorld, eigenTransform);


    for(auto cluster : clusters)
    {
      pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
      rs::ReferenceClusterPoints clusterpoints(cluster.points());
      rs::conversion::from(clusterpoints.indices(), *cluster_indices);

      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());


      for(std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
          pit != cluster_indices->indices.end(); pit++)
      {
        cluster_cloud->points.push_back(cloud_ptr->points[*pit]);
      }
      cluster_cloud->width = cluster_cloud->points.size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cluster_cloud, *output_cloud, indices);
        pcl::transformPointCloud (*output_cloud, *transformed_cloud, eigenTransform);


        pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>);

        /*std::cerr << "Cloud before projection: " << std::endl;
        for (size_t i = 0; i < transformed_cloud->points.size (); ++i)
          std::cerr << "    " << transformed_cloud->points[i].x << " "
                              << transformed_cloud->points[i].y << " "
                              << transformed_cloud->points[i].z << std::endl;*/

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;

        // Create the filtering object
        pcl::ProjectInliers<PointT> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (transformed_cloud);
        proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_projected);

        /*std::cerr << "Cloud after projection: " << std::endl;
        for (size_t i = 0; i < cloud_projected->points.size (); ++i)
          std::cerr << "    " << cloud_projected->points[i].x << " "
                              << cloud_projected->points[i].y << " "
                              << cloud_projected->points[i].z << std::endl;*/

        //cloud_to_img(cloud_projected,coords);
        cloud = cloud_projected;
        double radius,x,y;
        doMiniball(cloud,radius,x,y);
        percepteros::EnclosingCircle ec = rs::create<percepteros::EnclosingCircle>(tcas);

        rs::StampedPose pose = rs::create<rs::StampedPose>(tcas);
        cout << "x" << x << endl;
        cout << "y" << y << endl;
        vector<double> tr = {x,y,0};
        pose.frame.set("/iai_kitchen/counter_top_island_link");
        pose.translation.set(tr);
        ec.pose.set(pose);
        ec.radius.set(radius);
        ec.height.set(1);

        cluster.annotations.append(ec);

        outInfo("Cloud size: " << cloud_ptr->points.size());
        outInfo("took: " << clock.getTime() << " ms.");
    }
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RBBAnnotator)
