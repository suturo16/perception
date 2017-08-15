#pragma once

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>

//RS
#include <rs/types/all_types.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

//CATERROS
#include <percepteros/types/all_types.h>

// for the parser
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <assert.h>


struct regionDescriptor{
  //filled by parser
  std::string regionID;
  //std::vector<std::string> processViews;
  Eigen::Vector3d center_position;
  Eigen::Vector3d axis_ranges;

  //supported values computed later on
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr view_cloud_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr normal_ptr;
};


/**
* This Object is implemented as a Singleton.
*/
class ObjectRegionFilter{

private:
	static bool instanceFlag;
	static ObjectRegionFilter *single_orf;
	std::string region_file;
  std::vector<regionDescriptor> regions;

	ObjectRegionFilter():
		region_file("region-notes.yaml")
	{
		parseRegionConfig(this->region_file);
	}

	bool parseRegionConfig(std::string region_file);
	bool findRegion(std::string regionID, regionDescriptor& rD);
	void filterCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, float, float, std::string );
/*
	template <class PointType>
	void ObjectRegionFilter::filterCloud(pcl::PointCloud<PointType>::Ptr in_cloud_ptr, pcl::PointCloud<PointType>::Ptr out_cloud_ptr, float center, float range, std::string field_name);
*/
public:

	~ObjectRegionFilter()
	{
		instanceFlag = false;
	}

	static ObjectRegionFilter* getInstance()
	{
		if (!instanceFlag)
		{
			single_orf = new ObjectRegionFilter;
			instanceFlag = true;
			return single_orf;
		}
		else
		{
			return single_orf;
		}
	}
	
	bool getviewCloud(std::string regionID, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr view_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud);
/*
	template <class PointType> 
	bool ObjectRegionFilter::getCloud(std::string regionID, pcl::PointCloud<PointType>::Ptr view_cloud, pcl::PointCloud<PointType>::Ptr out_cloud);

*/

};


