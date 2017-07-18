#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
#include <rs/DrawingAnnotator.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <pcl/visualization/pcl_visualizer.h>


using namespace uima;


class SpatulaRecognition : public DrawingAnnotator
{
private:
  float test_param;

public:

  SpatulaRecognition(): DrawingAnnotator(__func__){

      ;//cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    outInfo("Test param =  " << test_param);
    cas.get(VIEW_CLOUD,*cloud_ptr);

    outInfo("Cloud size: " << cloud_ptr->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    ;
  }

};



// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SpatulaRecognition)