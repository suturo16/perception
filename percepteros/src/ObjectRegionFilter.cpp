#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

//RS
#include <rs/types/all_types.h>
#include <rs/DrawingAnnotator.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

//CATERROS
#include <percepteros/types/all_types.h>


using namespace uima;

class ObjectRegionFilter : public DrawingAnnotator
{
private:
  double pointSize;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr;


public:

  ObjectRegionFilter(): 
    DrawingAnnotator(__func__),
    pointSize(1),
    cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>)
  {
  }


  TyErrorId initialize(AnnotatorContext &ctx);
  TyErrorId destroy();
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec); 
  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);

};

TyErrorId ObjectRegionFilter::initialize(AnnotatorContext &ctx)
{
  outInfo("initialize");
  return UIMA_ERR_NONE;
}

TyErrorId ObjectRegionFilter::destroy()
{
  outInfo("destroy");
  return UIMA_ERR_NONE;
}

TyErrorId ObjectRegionFilter::processWithLock(CAS &tcas, ResultSpecification const &res_spec)
{
  outInfo("process start");
  rs::StopWatch clock;
  rs::SceneCas cas(tcas);
  cas.get(VIEW_CLOUD,*cloud_ptr);

  outInfo("Cloud width: " << cloud_ptr->width << ", height: " << cloud_ptr->height);

  return UIMA_ERR_NONE;
}

void ObjectRegionFilter::fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
{
  std::string cloudname("object region scene points");
  
  if (firstRun){
    visualizer.addPointCloud(cloud_ptr, cloudname);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
  } else {
    visualizer.updatePointCloud(cloud_ptr, cloudname);
    visualizer.removeAllShapes();
    visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
  }
}
// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ObjectRegionFilter)