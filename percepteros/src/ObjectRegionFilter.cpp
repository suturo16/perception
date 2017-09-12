#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>

//RS
#include <rs/types/all_types.h>
#include <rs/DrawingAnnotator.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

//CATERROS
#include <percepteros/types/all_types.h>
#include <percepteros/ObjectRegionFilter.h>

// for the parser
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <assert.h>

using namespace uima;

struct regionDescriptor{
  //filled by parser
  std::string regionID;
  std::vector<std::string> processViews;
  Eigen::Vector3d center_position;
  Eigen::Vector3d axis_ranges;

  //supported values computed later on
  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr normal_ptr;
};

class ObjectRegionFilter : public DrawingAnnotator
{
private:
  regionDescriptor currentRegion;
  std::string region_config;
  double pointSize;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr;
  std::vector<regionDescriptor> regions;

  bool parseRegionConfig(std::string);
  bool findRegion(std::string regionID, regionDescriptor& rD);
  void filterCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, float, float, std::string );
  bool getviewCloud(regionDescriptor rD, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr view_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud);

public:

  ObjectRegionFilter(): 
    DrawingAnnotator(__func__),
    pointSize(1)
  {
    cloud_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx);
  TyErrorId destroy();
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec); 
  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);

};

bool ObjectRegionFilter::parseRegionConfig(std::string region_file)
{
  std::string regionConfigFile = ros::package::getPath("percepteros") +"/config/" + region_file;
  YAML::Node regionList = YAML::LoadFile(regionConfigFile);
  if(!regionList.IsSequence())
  {
    outError("RegionDefinition: YAML contains no seqquence!");
    return false;
  }
  
  if(regionList.IsNull()){ 
    outError("Config file not found!");
    return false;
  } else {
    outInfo("Config file found at: " + regionConfigFile);
  }

  for(YAML::const_iterator reg_it = regionList.begin(); reg_it != regionList.end(); ++reg_it){
    regionDescriptor rD;
    const YAML::Node& regionItem = *reg_it;
    rD.regionID = regionItem["regionID"].as<std::string>();
    outInfo("processing region with ID" + rD.regionID);

    YAML::Node viewsToProcess = regionItem["viewsToProcess"];
    assert(viewsToProcess.IsSequence());
    if(viewsToProcess.size()==0)
    {
      outError("Parser does not accept region with 0 views!");
      return false;
    }
    for(YAML::const_iterator view_it = viewsToProcess.begin(); view_it != viewsToProcess.end(); ++view_it){
      std::string viewName = view_it->as<std::string>();
      rD.processViews.push_back(viewName);
    }

    YAML::Node region_center = regionItem["region_center"];
    assert(region_center.IsSequence());
    if (region_center.size() != 3)
    {
      outError("ParserError: region_center needs exactly 3 elements");
      return false;
    }
    rD.center_position = Eigen::Vector3d(region_center[0].as<double>(), region_center[1].as<double>(), region_center[2].as<double>());
    
    YAML::Node range = regionItem["range"];
    assert(range.IsSequence());
    if (range.size() != 3)
    {
      outError("ParserError: range needs exactly 3 elements");
      return false;
    }
    rD.axis_ranges = Eigen::Vector3d(range[0].as<double>(), range[1].as<double>(), range[2].as<double>());

    this->regions.push_back(rD);
  }
  return true;
}

bool ObjectRegionFilter::findRegion(std::string regionID, regionDescriptor& rD)
{
  for (size_t i = 0; i < this->regions.size(); i++) {
    outInfo("iterating through: " << regions[i].regionID);
    if (regions[i].regionID == regionID) 
    {
      rD = regions[i];
      outInfo("found Region");
      return true;
    }
  }
  outError("found no region!");
  return false;
}

void ObjectRegionFilter::filterCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud_ptr, float center, float range, std::string field_name)
{
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setKeepOrganized(true);
  pass.setInputCloud (in_cloud_ptr);
  //outInfo("in cloud size = " << in_cloud_ptr->size());
  //outInfo("field_name = " << field_name);
  pass.setFilterFieldName (field_name);
  float left_endpoint = center - (range/2);
  float right_endpoint = center + (range/2);
  //outInfo("setting filter Limits le = " << left_endpoint << "right endpoint = " << right_endpoint);
  pass.setFilterLimits (left_endpoint, right_endpoint);
  //outInfo("filtering");
  pass.filter (*out_cloud_ptr);
  //outInfo("end filterCloud");
  /*
  */
}


bool ObjectRegionFilter::getviewCloud(regionDescriptor rD, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr view_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud)
{

  outInfo(FG_MAGENTA << "filter 1");
  filterCloud(view_cloud, out_cloud, rD.center_position.x(), rD.axis_ranges.x(), std::string("x"));
  outInfo(FG_MAGENTA << "filter 2");
  filterCloud(out_cloud,  out_cloud, rD.center_position.y(), rD.axis_ranges.y(), "y");
  outInfo(FG_MAGENTA << "filter 3");
  filterCloud(out_cloud,  out_cloud, rD.center_position.z(), rD.axis_ranges.z(), "z");
  /*
*/
  return true;
}

TyErrorId ObjectRegionFilter::processWithLock(CAS &tcas, ResultSpecification const &res_spec)
{
  outInfo(FG_MAGENTA << "process start");
  //cloud_ptrs.clear();

  rs::StopWatch clock;
  rs::SceneCas cas(tcas);
  rs::Scene scene = cas.getScene();

  std::vector<percepteros::PipelineAnnotation> pipeline_spec;
  scene.identifiables.filter(pipeline_spec);
  /*
  if (pipeline_spec.size() == 0) 
  {
    outError(FG_MAGENTA << "Scene contains no PipelineAnnotation!\nStopping process.");
    return UIMA_ERR_NONE;
  }
  std::string pipelineID = pipeline_spec[0].pipelineID.get();
  */
  std::string pipelineID = PipelineIdentifikator::pipelineID;
  if (pipelineID == "")
  {
    outError(FG_MAGENTA << "pipelineID is empty");
    return UIMA_ERR_NONE;
  }

  outInfo(FG_MAGENTA << "pipelineID set to: " << pipelineID);

  findRegion(pipelineID, currentRegion);

  for (int i = 0; i < currentRegion.processViews.size(); i++)
  {
    outInfo(FG_MAGENTA << "Wubbalubbadubdub " << i << 0);
    cas.get(currentRegion.processViews[i].c_str() ,*cloud_ptr);
    outInfo(FG_MAGENTA << "Wubbalubbadubdub " << i << 1);
    getviewCloud(currentRegion, cloud_ptr, cloud_ptr);
    cas.set(currentRegion.processViews[i].c_str() ,*cloud_ptr);
    outInfo(FG_MAGENTA << "terminated getviewCloud successfully");
  }



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
  visualizer.addSphere(pcl::PointXYZ(currentRegion.center_position.x(), currentRegion.center_position.y(), currentRegion.center_position.z()), 0.1, "RegionSpots");
}

TyErrorId ObjectRegionFilter::initialize(AnnotatorContext &ctx)
{
  outInfo(FG_MAGENTA << "initialize");
  if(ctx.isParameterDefined("region_config")) ctx.extractValue("region_config", region_config);
  parseRegionConfig(region_config);
  
  return UIMA_ERR_NONE;
}

TyErrorId ObjectRegionFilter::destroy()
{
  outInfo("destroy");
  return UIMA_ERR_NONE;
}

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ObjectRegionFilter)