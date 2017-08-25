#include <percepteros/RegionDefinition.h>


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

//    YAML::Node viewsToProcess = regionItem["viewsToProcess"];
//    assert(viewsToProcess.IsSequence());
//    if(viewsToProcess.size()==0)
//    {
//      outError("Parser does not accept region with 0 views!");
//      return false;
//    }
//    for(YAML::const_iterator view_it = viewsToProcess.begin(); view_it != viewsToProcess.end(); ++view_it){
//      std::string viewName = view_it->as<std::string>();
//      rD.processViews.push_back(viewName);
//    }

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
  pass.setInputCloud (in_cloud_ptr);
  outInfo("in cloud size = " << in_cloud_ptr->size());
  outInfo("field_name = " << field_name);
  pass.setFilterFieldName (field_name);
  float left_endpoint = center - (range/2);
  float right_endpoint = center + (range/2);
  outInfo("setting filter Limits le = " << left_endpoint << "right endpoint = " << right_endpoint);
  pass.setFilterLimits (left_endpoint, right_endpoint);
  outInfo("filtering");
  pass.filter (*out_cloud_ptr);
  outInfo("end filterCloud");
	/*
  */
}

bool ObjectRegionFilter::getviewCloud(std::string regionID, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr view_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud)
{
	regionDescriptor rD;
	if (!findRegion(regionID, rD)) 
	{
		outError("Abort: found no matching region");
		return false;
	}

	filterCloud(view_cloud, out_cloud, rD.center_position.x(), rD.axis_ranges.x(), std::string("x"));
	filterCloud(out_cloud,  out_cloud, rD.center_position.y(), rD.axis_ranges.y(), "y");
	filterCloud(out_cloud,  out_cloud, rD.center_position.z(), rD.axis_ranges.z(), "z");

	return true;
}

/*

template <class PointType>
bool ObjectRegionFilter::getCloud(std::string regionID, pcl::PointCloud<PointType>::Ptr view_cloud, pcl::PointCloud<PointType>::Ptr out_cloud)
{
	regionDescriptor rD;
	if (!findRegion(regionID, rD)) 
	{
		outError("Abort: found no matching region");
		return false;
	}

	pcl::PointCloud<PointType>::Ptr tmp0(pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr tmp1(pcl::PointCloud<PointType>);

	filterCloud(view_cloud, tmp0, rD.center_position.x(), rD.axis_ranges.x(), "x");
	filterCloud(tmp0, tmp1, rD.center_position.y(), rD.axis_ranges.y(), "y");
	filterCloud(tmp1, out_cloud, rD.center_position.z(), rD.axis_ranges.z(), "z");


	return true;
}

template <class PointType>
void ObjectRegionFilter::filterCloud(pcl::PointCloud<PointType>::Ptr in_cloud_ptr, pcl::PointCloud<PointType>::Ptr out_cloud_ptr, float center, float range, std::string field_name)
{
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud (in_cloud_ptr);
  pass.setFilterFieldName (field_name);
  float left_endpoint = center - (range/2);
  float right_endpoint = center + (range/2);
  pass.setFilterLimits (left_endpoint, right_endpoint);
  pass.filter (*out_cloud_ptr);
}
*/