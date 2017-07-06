#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>
#include <rs/utils/time.h>

#include <percepteros/types/all_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/surface/convex_hull.h>

#include <cmath>
#include <sstream>
#include <typeinfo>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;

class BoardAnnotator : public DrawingAnnotator
{
private:
	PCR::Ptr cloud = PCR::Ptr(new PCR);
	PCR::Ptr cluster = PCR::Ptr(new PCR);
	PCR hull;
	std::vector<PCR::Ptr> hulls;

public:

  BoardAnnotator(): DrawingAnnotator(__func__){
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

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
	
	//empty containers
	cloud->clear();
	cluster->clear();
	hull.clear();
	hulls.clear();

	//get clusters
    rs::SceneCas cas(tcas);
	rs::Scene scene = cas.getScene();
	std::vector<rs::Cluster> clusters;
	scene.identifiables.filter(clusters);

	//get scene points
	cas.get(VIEW_CLOUD, *cloud);

	//prepare segmenter
	pcl::SACSegmentation<PointR> seg;
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(500);
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);
	seg.setDistanceThreshold(0.005);
	seg.setRadiusLimits(0.1, 0.15);
	seg.setInputCloud(cluster);

	//prepare extractor
	pcl::ExtractIndices<PointR> ex;
	ex.setInputCloud(cloud);

	pcl::ConvexHull<PointR> huller;

	for (auto it = clusters.begin(); it != clusters.end(); ++it) {
		//extract cluster into point cloud
		auto rclust = *it; cluster->clear();
		pcl::PointIndices::Ptr indices(new pcl::PointIndices);
		rs::conversion::from(((rs::ReferenceClusterPoints)rclust.points.get()).indices.get(), *indices);
		ex.setIndices(indices);
		ex.filter(*cluster);
		
		hull = *(new PCR);

		//create convex hull
		huller.setInputCloud(cluster);
		huller.reconstruct(hull);
		boost::shared_ptr<PCR> hull_ptr = boost::make_shared<PCR>(hull);
		hulls.push_back(hull_ptr);
		/*
		//segment
		indices->indices.clear();
		pcl::ModelCoefficients::Ptr cco(new pcl::ModelCoefficients);
		seg.segment(*indices, *cco);

		outInfo("Found a circle with the following radius: " << cco->values[3]);
		*/
	}
	return UIMA_ERR_NONE;
  }

void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
		  visualizer.removeAllPointClouds();
		  for (PCR::Ptr hull : hulls) {
		  	visualizer.addPointCloud(hull);
		  }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BoardAnnotator)
